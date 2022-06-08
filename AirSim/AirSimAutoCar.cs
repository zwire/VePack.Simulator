using System;
using System.Linq;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using System.Reactive.Linq;
using System.Reactive.Threading.Tasks;
using Microsoft.Extensions.Configuration;
using VePack;
using VePack.Utilities;
using VePack.Utilities.Geometry;
using VePack.Utilities.IO;
using VePack.Utilities.NeuralNetwork;
using VePack.Plugin.Navigation;
using VePack.Plugin.Controllers.ModelFree;
using VePack.Plugin.Controllers.ModelBased.Steering;

namespace AirSim
{
    public sealed class AirSimAutoCar : IVehicle<CarInformation>
    {

        // ------ fields ------ //

        private readonly Rootobject _config;
        private readonly AirSimConnector _car;
        private readonly Process _python;
        private readonly TcpSocketClient _client;
        private readonly BidirectionalDataStream _stream;
        private readonly Pid _speedController;
        private readonly GeometricSteeringModel _steerModel;
        private readonly ISteeringController _steerController;
        private readonly IDisposable _connector;
        private readonly bool _autoSteering;
        private double _targetSpeed;
        private MapNavigator _navigator;
        private CarOperation _operation;
        private CancellationTokenSource _cts;


        // ------ properties ------ //

        public IObservable<CompositeInfo<CarInformation>> InfoUpdated { get; }


        // ------ constructors ------ //

        public AirSimAutoCar()
        {
            _config = new ConfigurationBuilder().AddJsonFile("rootsettings.json").Build().Get<Rootobject>();
            _cts = new();
            _cts.Cancel();
            _python = new() { StartInfo = new(_config.PythonExe) { Arguments = _config.PyFile} };
            _python.Start();
            _client = new("127.0.0.1", 3000);
            _stream = _client.GetStream();
            _car = new(_stream);
            _operation = new();
            _autoSteering = _config.AutoSteering;
            _speedController = new(PidType.Speed, 0.001, 0, 0.003);
            _steerModel = _config.UseNN 
                ? new NnSteeringModel(
                    NetworkGraph.Load(_config.SteeringModelFile),
                    _config.TrainModel ? 4 : 0,
                    0.1
                ) 
                : new GeometricSteeringModel(1.0, 1.0, 0.1);

            _steerController = new PfcSteeringController(
                _steerModel,
                _config.PfcResponseGain,
                new[]
                {
                    _config.PfcFirstCoincidenceIndex * 3,
                    _config.PfcFirstCoincidenceIndex * 4,
                    _config.PfcFirstCoincidenceIndex * 5
                },
                Angle.FromDegree(35),
                Angle.FromDegree(10)
            );

            //_steerController = new LqrSteeringController(
            //    _steerModel,
            //    0.1, 1, 1, 1,
            //    Angle.FromDegree(35),
            //    Angle.FromDegree(10)
            //);

            var line = "";
            if (_config.MapFile is not null && _config.MapFile is not "")
            {
                SetMap(_config.MapFile);
                var iniPoint = _navigator.MapData.Paths[0].Points[0];
                foreach (var path in _navigator.MapData.Paths)
                    foreach (var p in path.Points)
                        line += $"{p.Y - iniPoint.Y},{p.X - iniPoint.X},";
            }
            _stream.WriteString(line);
            _car.ConnectSendingStream(new Freq(100));

            var observable = _car.ConnectReceivingStream(new Freq(10))
                .Finally(() => Dispose())
                .Select(x =>
                {
                    var wgs = new WgsPointData(x.Gnss.Latitude, x.Gnss.Longitude);
                    return new CompositeInfo<CarInformation>(
                        _cts is not null && _cts.IsCancellationRequested is false,
                        x, x.Gnss, x.Imu, _navigator?.Update(wgs.ToUtm(), x.Imu.Yaw)
                    );
                })
                .TakeWhile(x =>
                {
                    if (x?.Geo is not null)
                    {
                        return true;
                    }
                    else
                    {
                        Dispose();
                        return false;
                    }
                })
                .Publish();
            _connector = observable.Connect();
            InfoUpdated = observable;
        }


        // ------ public methods ------ //

        public void Dispose()
        {
            Stop();
            _connector?.Dispose();
            _car.Dispose();
            _stream.Dispose();
            _client.Dispose();
            _python.Dispose();
        }

        public void SetMap(string mapFile, string plnFile = null)
        {
            var map = NaviHelper.LoadMapFromFile(mapFile);
            if (plnFile is not null && plnFile is not "")
                map = NaviHelper.ModifyMapByPlnFile(plnFile, map);
            _navigator = new(map);
            _navigator.CurrentPathChanged.Finally(() => Dispose());
        }

        public async void Start()
        {

            _cts = new();
            _operation = new();

            InfoUpdated
               .Where(x => x?.Geo is not null && x?.Vehicle is not null)
               .Where(_ => _operation.FootBrake is 0)
               .TakeUntil(x => _cts.IsCancellationRequested)
               .Do(x =>
               {
                   var speedError = x.Vehicle.VehicleSpeed - _targetSpeed;
                   _operation.Throttle += _speedController.GetControlQuantity(speedError).InsideOf(-1, 1);
                   _car.Set(_operation);
               })
               .Subscribe();

            while (_autoSteering && !_cts.IsCancellationRequested)
            {
                if (await IsToTurnAsync())
                    await TurnAsync(_cts.Token);
                await FollowAsync(_cts.Token);
            }

        }

        public void Stop()
        {
            _operation = new() { FootBrake = 2 };
            _car.Set(_operation);
            if (!_cts.IsCancellationRequested)
                InfoUpdated.Where(x => x is not null).TakeUntil(x => x.Vehicle.VehicleSpeed < 0.1).ToTask().Wait();
            _cts?.Cancel();
        }

        public void SetVehicleSpeed(double speed)
        {
            _targetSpeed = speed;
            _operation.FootBrake = 0;
            _car.Set(_operation);
        }

        public void SetSteeringAngle(Angle steerAngle)
        {
            _operation.SteeringAngle = Angle.FromDegree(steerAngle.Degree.InsideOf(-35, 35));
            _car?.Set(_operation);
        }


        // ------private methods ------ //

        private void SetBrake(int level)
        {
            _operation.Throttle = 0;
            _operation.FootBrake = level;
            _car.Set(_operation);
        }

        private async Task<CompositeInfo<CarInformation>> FollowAsync(CancellationToken ct)
        {
            Console.WriteLine($"\nstart to follow path {_navigator.CurrentPathIndex}.\n");
            if (_navigator.CurrentPath.Id is "Work" && _config.PathEndMargin > 0)
                _navigator.CurrentPath.ExtendLast(_config.PathEndMargin);
            return await InfoUpdated
                .Where(x => x?.Geo is not null && x?.Vehicle is not null)
                .TakeWhile(x => !ct.IsCancellationRequested)
                .TakeUntil(_navigator.CurrentPathChanged)
                .Do(x =>
                {
                    var lateral = x.Geo.LateralError;
                    var heading = x.Geo.HeadingError;
                    var steer = x.Vehicle.SteeringAngle;
                    var speed = x.Vehicle.VehicleSpeed / 3.6;
                    double.TryParse(_navigator.CurrentPoint.Id, out var curvature);
                    _steerModel.UpdateA(lateral, heading, steer, speed, curvature);
                    var angle = _steerController.GetSteeringAngle(lateral, heading, steer);
                    SetSteeringAngle(angle);
                    Console.Write($"Steer: {angle.Degree:f1} ... ");
                })
                //.Finally(() => _steerModel.SaveModel("latest.ynn"))
                .ToTask();
        }

        private async Task<bool> IsToTurnAsync()
        {
            return Math.Abs((await InfoUpdated.FirstOrDefaultAsync()).Geo.HeadingError.Degree) > 90;
        }

        private async Task TurnAsync(CancellationToken ct)
        {
            // 先にパスを得る
            var paths = NaviHelper.GenerateTurnPath(
                _navigator.MapData.Paths[_navigator.CurrentPathIndex - 1],
                _navigator.CurrentPath,
                _config.TurnRadius
            );
            // 前半パスを追従
            _navigator.InsertPath(_navigator.CurrentPathIndex, paths.FirstHalf);
            await FollowAsync(ct);
            // バックが必要なら
            if (paths.BackPath is not null)
            {
                SetSteeringAngle(Angle.Zero);
                SetBrake(2);
                await Task.Delay(200, ct);
                SetVehicleSpeed(-_targetSpeed);
                //_navigator.InsertPath(_navigator.CurrentPathIndex, paths.BackPath);
                //await FollowAsync(ct);
                // secondの始点前までバック
                var seg = paths.SecondHalf.GetSegment(0);
                await InfoUpdated.TakeWhile(x => x.Geo.VehiclePosition.IsInFrontOf(seg)).ToTask(ct);
                SetBrake(2);
                await Task.Delay(200, ct);
                SetVehicleSpeed(-_targetSpeed);
            }
            // 後半パスを追従
            _navigator.InsertPath(_navigator.CurrentPathIndex, paths.SecondHalf);
            await FollowAsync(ct);
        }

    }
}
