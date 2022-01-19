using System;
using System.Linq;
using System.Diagnostics;
using System.Reactive.Linq;
using System.Reactive.Threading.Tasks;
using System.Threading;
using System.Threading.Tasks;
using VePack;
using VePack.Utilities;
using VePack.Plugin.Controllers;
using VePack.Plugin.Navigation;

namespace AirSim
{
    public class AirSimAutoCar : IAutoVehicle<CarInformation>
    {

        // ------ fields ------ //

        private const string pythonExe = "..\\..\\..\\..\\..\\..\\..\\AppData\\Local\\Programs\\Python\\Python37\\python.exe";
        private const string pyFile = "..\\..\\..\\..\\AirSim\\airsim_server.py";
        private readonly AirSimConnector _car;
        private readonly Process _python;
        private readonly TcpSocketClient _client;
        private readonly BidirectionalDataStream _stream;
        private readonly Pid _speedController;
        private readonly PurePursuit _steerController;
        private readonly IDisposable _connector;
        private double _targetSpeed;
        private MapNavigator _navigator;
        private CarOperation _operation;
        private CancellationTokenSource _cts;


        // ------ properties ------ //

        public IObservable<CompositeInfo<CarInformation>> InfoUpdated { get; }


        // ------ constructors ------ //

        public AirSimAutoCar(string mapFile = null, string plnFile = null)
        {
            _python = new() { StartInfo = new(pythonExe) { Arguments = pyFile } };
            _python.Start();
            _client = new("127.0.0.1", 3000);
            _stream = _client.GetStream();
            _car = new(_stream);
            _speedController = new(PidType.Speed, 0.002, 0, 0.003);
            _steerController = new(2.0);
            _operation = new();

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
                    var position = MapHelper.WgsToUtm(wgs) - new Vector2D(Math.Sin(heading.Radian), Math.Cos(heading.Radian));
                    var lookAheadDistance = Math.Abs(x.VehicleSpeed) + 1;
                    var geoInfo = _navigator?.Update(position, heading, lookAheadDistance);
                    return new CompositeInfo<CarInformation>(
                        _cts is not null && _cts.IsCancellationRequested is false,
                        x, x.Gnss, x.Imu, geoInfo
                    );
                })
                .TakeUntil(x => x?.Geo is null)
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
            var map = MapHelper.LoadMapFromFile(mapFile);
            if (plnFile is not null)
                map = MapHelper.ModifyMapByPlnFile(plnFile, map);
            _navigator = new(map, 3);
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

            while (!_cts.IsCancellationRequested)
            {
                if (await IsToTurnAsync())
                    await TurnAsync(_cts.Token);
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


        // ------ private methods ------ //

        private void SetSteeringAngle(Angle angle)
        {
            _operation.SteeringAngle = new(angle.Degree.InsideOf(-30, 30), AngleType.Degree);
            _car?.Set(_operation);
        }

        private void SetBrake(int level)
        {
            _operation.Throttle = 0;
            _operation.FootBrake = level;
            _car.Set(_operation);
        }

        private async Task FollowAsync(CancellationToken ct)
        {
            await InfoUpdated
                .Where(x => x?.Geo is not null && x?.Vehicle is not null)
                .TakeUntil(x => ct.IsCancellationRequested)
                .TakeUntil(_navigator.CurrentPathChanged)
                .Do(x =>
                {
                    var current = x.Geo.VehiclePosition;
                    var heading = x.Geo.VehicleHeading;
                    var target = x.Geo.LookAheadPoint;
                    SetSteeringAngle(_steerController.GetSteeringAngle(current, heading, target));
                })
                .ToTask();
        }

        private async Task<bool> IsToTurnAsync()
        {
            var vehicleDirection = (await InfoUpdated.TakeUntil(x => x is not null).LastOrDefaultAsync()).Geo.VehicleHeading;
            var pathDirection = new Vector2D(_navigator.CurrentPath.Points[0], _navigator.CurrentPath.Points[1]).ClockwiseAngleFromY;
            return Math.Abs((vehicleDirection - pathDirection).Degree) > 90;
        }

        private async Task TurnAsync(CancellationToken ct)
        {

            var initialInfo = await InfoUpdated.TakeUntil(x => x is not null).LastOrDefaultAsync();

            // 今のところ変形圃場は無理です
            var initialHeading = initialInfo.Geo.VehicleHeading;
            var initialPoint = initialInfo.Geo.VehiclePosition;
            var targetPoint = initialInfo.Geo.ReferencePoint;
            var desiredDirection = new Vector2D(initialPoint, targetPoint).ClockwiseAngleFromY;
            var left = desiredDirection - initialHeading < Angle.Zero;
            SetSteeringAngle(new(left ? -30 : 30, AngleType.Degree));

            // はじめの85度
            var judgePoint = await InfoUpdated
                .Where(x => x?.Geo is not null)
                .TakeUntil(x => left
                    ? desiredDirection - x.Geo.VehicleHeading > new Angle(-5, AngleType.Degree)
                    : desiredDirection - x.Geo.VehicleHeading < new Angle(+5, AngleType.Degree)
                )
                .Select(x => x.Geo.VehiclePosition)
                .LastOrDefaultAsync();

            // 直進 or スイッチバック
            SetSteeringAngle(Angle.Zero);
            var minTurnLength = judgePoint.DistanceTo(initialPoint);
            var switchback = minTurnLength > judgePoint.DistanceTo(targetPoint);
            if (switchback)
            {
                SetBrake(2);
                await Task.Delay(1000);
                SetVehicleSpeed(-Math.Abs(_targetSpeed));
                await InfoUpdated.Where(x => x is not null).TakeUntil(x => x.Geo.VehiclePosition.DistanceTo(targetPoint) > minTurnLength).ToTask();
                SetBrake(2);
                await Task.Delay(1000);
                SetVehicleSpeed(Math.Abs(_targetSpeed));
            }
            else
            {
                await InfoUpdated.Where(x => x is not null).TakeUntil(x => x.Geo.VehiclePosition.DistanceTo(targetPoint) > minTurnLength).ToTask();
            }

            // 後半
            desiredDirection = new Vector2D(_navigator.CurrentPath.Points[0], _navigator.CurrentPath.Points[1]).ClockwiseAngleFromY;
            SetSteeringAngle(new(left ? -25 : 25, AngleType.Degree));
            await InfoUpdated
                .Where(x => x?.Geo is not null)
                .TakeUntil(x =>
                    left
                       ? desiredDirection - x.Geo.VehicleHeading > new Angle(-10, AngleType.Degree)
                       : desiredDirection - x.Geo.VehicleHeading < new Angle(+10, AngleType.Degree)
                )
                .ToTask();
        }

    }
}
