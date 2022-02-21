using System;
using System.Linq;
using System.Diagnostics;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using System.Reactive.Linq;
using System.Reactive.Threading.Tasks;
using VePack;
using VePack.Utilities;
using VePack.Plugin.Navigation;
using VePack.Plugin.Controllers;
using VePack.Plugin.Controllers.NeuralNetwork;

namespace AirSim
{
    public class AirSimAutoCar : IAutoVehicle<CarInformation>
    {

        // ------ fields ------ //

        private const string pythonExe = "..\\..\\..\\..\\..\\..\\..\\AppData\\Local\\Programs\\Python\\Python37\\python.exe";
        private const string pyFile = "..\\..\\..\\..\\AirSim\\airsim_server.py";
        private const double _margin = 1.0;
        private readonly AirSimConnector _car;
        private readonly Process _python;
        private readonly TcpSocketClient _client;
        private readonly BidirectionalDataStream _stream;
        private readonly Pid _speedController;
        //private readonly NNSteeringModel _steerModel;
        private readonly KinematicSteeringModel _steerModel;
        private readonly PfcSteeringController _steerController;
        private readonly IDisposable _connector;
        private readonly bool _autoSteering;
        private double _targetSpeed;
        private MapNavigator _navigator;
        private CarOperation _operation;
        private CancellationTokenSource _cts;


        // ------ properties ------ //

        public IObservable<CompositeInfo<CarInformation>> InfoUpdated { get; }


        // ------ constructors ------ //

        public AirSimAutoCar(string mapFile = null, string plnFile = null, bool autoSteering = true)
        {
            _python = new() { StartInfo = new(pythonExe) { Arguments = pyFile } };
            _python.Start();
            _client = new("127.0.0.1", 3000);
            _stream = _client.GetStream();
            _car = new(_stream);
            _operation = new();
            _autoSteering = autoSteering;
            _speedController = new(PidType.Speed, 0.001, 0, 0.003);
            //var net = NetworkGraph.Load("net.json");
            //_steerModel = new NNSteeringModel(net, 1.0, 1.0, 0.1);
            _steerModel = new KinematicSteeringModel(1.0, 1.0, 0.1);
            _steerController = new(
                _steerModel,
                new[] { 7, 10, 12 }, 
                1.0,
                new(35, AngleType.Degree),
                new(10, AngleType.Degree)
            );

            var line = "";
            if (mapFile is not null)
            {
                SetMap(mapFile, plnFile);
                var iniPoint = _navigator.MapData.Paths[0].Points[0];
                foreach (var path in _navigator.MapData.Paths)
                    foreach (var p in path.Points)
                        line += $"{p.Y - iniPoint.Y},{p.X - iniPoint.X},";
            }
            _stream.WriteString(line);
            _car.ConnectSendingStream(TimeSpan.FromMilliseconds(100));
            var observable = _car.ConnectReceivingStream(TimeSpan.FromMilliseconds(100))
                .Finally(() => Dispose())
                .Select(x =>
                {
                    var wgs = new WgsPointData(x.Gnss.Latitude, x.Gnss.Longitude);
                    var heading = x.Imu.Yaw;
                    var position = NaviHelper.WgsToUtm(wgs);
                    var geoInfo = _navigator?.Update(position, heading, 1e-3);
                    return new CompositeInfo<CarInformation>(
                        _cts is not null && _cts.IsCancellationRequested is false,
                        x, x.Gnss, x.Imu, geoInfo
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
            _cts?.Dispose();
            _connector?.Dispose();
            _car.Dispose();
            _stream.Dispose();
            _client.Dispose();
            _python.Dispose();
        }

        public void SetMap(string mapFile, string plnFile = null)
        {
            var map = NaviHelper.LoadMapFromFile(mapFile);
            if (plnFile is not null)
                map = NaviHelper.ModifyMapByPlnFile(plnFile, map);
            _navigator = new(map, 1e-3);
            _navigator.CurrentPathChanged.Finally(() => Dispose());
        }

        public async void Start()
        {

            _cts = new();
            _operation = new();

            InfoUpdated
               .Where(x => x?.Geo is not null && x?.Vehicle is not null)
               .Where(_ => _operation.FootBrake == 0)
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
                    await SwitchBackAsync();
                await FollowAsync(_cts.Token);
            }

        }

        public void Stop()
        {
            _cts?.Cancel();
            _operation = new() { FootBrake = 2 };
            _car.Set(_operation);
            InfoUpdated.Where(x => x is not null).TakeUntil(x => x.Vehicle.VehicleSpeed < 0.1).ToTask().Wait();
        }

        public void SetVehicleSpeed(double speed)
        {
            _targetSpeed = speed;
            _operation.FootBrake = 0;
            _car.Set(_operation);
        }

        public void SetSteeringAngle(Angle angle)
        {
            _operation.SteeringAngle = new(angle.Degree.InsideOf(-35, 35), AngleType.Degree);
            _car?.Set(_operation);
        }


        // ------ private methods ------ //

        private void SetBrake(int level)
        {
            _operation.Throttle = 0;
            _operation.FootBrake = level;
            _car.Set(_operation);
        }

        private async Task<UtmPathData> GenerateTurnPath(UtmPathData passedPath, UtmPathData targetPath)
        {
            targetPath = NaviHelper.ModifyPathDirection(targetPath, passedPath.Points[^1]);
            var current = await InfoUpdated.FirstOrDefaultAsync();
            var entranceDirectionVector = targetPath.GetEntranceDirectionVector().UnitVector;
            var hypotenuseVector = new Vector2D(current.Geo.VehiclePosition, targetPath.Points[0] - entranceDirectionVector * _margin);
            var interiorAngle = hypotenuseVector.GetClockwiseAngleFrom(entranceDirectionVector);
            var radius = Math.Abs(hypotenuseVector.Length * Math.Cos(interiorAngle.Radian));
            var curvature = 1.0 / radius;
            var orthogonalDirectionVector = new Vector2D(passedPath.Points[^1], targetPath.Points[0]).UnitVector;
            var end = targetPath.Points[0] - entranceDirectionVector * _margin;
            var left = entranceDirectionVector.GetClockwiseAngleFrom(orthogonalDirectionVector) < Angle.Zero;
            var arc = new Arc2D(end, end - orthogonalDirectionVector * radius,
                new(left ? -90 : 90, AngleType.Degree)).GetReverse().ApproximateAsPoints(new(10, AngleType.Degree));
            var list = new List<UtmPointData>();
            list.Add(new(arc[0] - orthogonalDirectionVector * 0.1));
            list.AddRange(arc.Select(a => new UtmPointData(a)));
            list.Add(new(end + entranceDirectionVector * 0.1));
            return new(list, left ? $"{-curvature}" : $"{curvature}");
        }

        private async Task FollowAsync(CancellationToken ct)
        {
            Console.WriteLine($"\nstart to follow path {_navigator.CurrentPathIndex}.\n");
            var curvature = 0.0;
            double.TryParse((string)_navigator.CurrentPath.Id, out curvature);
            await InfoUpdated
                .Where(x => x?.Geo is not null && x?.Vehicle is not null)
                .TakeWhile(x => !ct.IsCancellationRequested)
                .TakeUntil(_navigator.CurrentPathChanged)
                .Do(x =>
                {
                    var lateral = x.Geo.LateralError;
                    var heading = x.Geo.HeadingError;
                    var steer = x.Vehicle.SteeringAngle;
                    var speed = x.Vehicle.VehicleSpeed / 3.6;
                    _steerModel.SetParams(speed, x.Geo.OnThePath ? curvature : 0);
                    //_steerModel.PredictNext(lateral, heading, steer, speed, curvature);
                    _steerController.ResetParams();
                    var angle = _steerController.GetSteeringAngle(lateral, heading, steer);
                    SetSteeringAngle(angle);
                    Console.Write($"Steer: {angle.Degree:f1} ... ");
                })
                .ToTask();
        }

        private async Task<bool> IsToTurnAsync()
        {
            return Math.Abs((await InfoUpdated.FirstOrDefaultAsync()).Geo.HeadingError.Degree) > 90;
        }

        private async Task SwitchBackAsync()
        {
            var ckpt1 = await InfoUpdated.FirstOrDefaultAsync();
            var left = ckpt1.Geo.LateralError < 0;
            await InfoUpdated.TakeUntil(x => x.Geo.VehiclePosition.DistanceTo(ckpt1.Geo.VehiclePosition) > _margin);
            SetSteeringAngle(new(left ? -30 : 30, AngleType.Degree));
            var targetPath = _navigator.CurrentPath;
            var entranceDirection = targetPath.GetEntranceDirectionVector().ClockwiseAngleFromY;
            var ckpt2 = await InfoUpdated.TakeUntil(x => Math.Abs(x.Geo.HeadingError.Degree) < 95).ToTask();
            SetBrake(2);
            await Task.Delay(100);
            SetBrake(0);
            SetSteeringAngle(Angle.Zero);
            SetVehicleSpeed(-_targetSpeed);
            var radius = ckpt1.Geo.LateralError - ckpt2.Geo.LateralError;
            await InfoUpdated.TakeUntil(x => left ? x.Geo.LateralError < radius : x.Geo.LateralError > radius).ToTask();
            SetBrake(2);
            await Task.Delay(100);
            var turnPath = await GenerateTurnPath(_navigator.MapData.Paths[_navigator.CurrentPathIndex - 1], _navigator.CurrentPath);
            _navigator.InsertPath(_navigator.CurrentPathIndex, turnPath);
            SetBrake(0);
            SetVehicleSpeed(-_targetSpeed);
        }

    }
}
