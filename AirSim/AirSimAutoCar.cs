using System;
using System.Diagnostics;
using System.Reactive.Linq;
using System.Reactive.Threading.Tasks;
using System.Threading;
using System.Threading.Tasks;
using VePack;
using VePack.Utilities;
using VePack.Plugin.Controllers;
using VePack.Plugin.Navigation;
using System.Collections.Generic;

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
                    var position = MapHelper.WgsToUtm(wgs);
                    var heading = x.Imu.Yaw;
                    var lookAheadDistance = Math.Abs(x.VehicleSpeed) + 1;
                    var geoInfo = _navigator?.Update(position, heading, lookAheadDistance);
                    return new CompositeInfo<CarInformation>(
                        _cts is not null && _cts.IsCancellationRequested is false,
                        x, x.Gnss, x.Imu, geoInfo
                    );
                })
                .Publish();
            _connector = observable.Connect();
            InfoUpdated = observable;
        }


        // ------ public methods ------ //

        public void Dispose()
        {
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
            _navigator = new(map, 5);
            _navigator.CurrentPathChanged.Finally(() => Dispose());
        }

        public async void Start()
        {
            _cts = new();
            _operation = new();
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
        }

        public void SetVehicleSpeed(double speed)
        {
            _targetSpeed = speed;
        }


        // ------ private methods ------ //

        private async Task FollowAsync(CancellationToken ct)
        {
            Debug.WriteLine("start following.");
            await InfoUpdated
               .Where(x => x is not null)
               .TakeUntil(x => ct.IsCancellationRequested)
               .TakeUntil(_navigator.CurrentPathChanged)
               .Do(x =>
               {
                   var actualSpeed = x.Vehicle.VehicleSpeed;
                   var speedError = actualSpeed - _targetSpeed;
                   _operation.Throttle += _speedController.GetControlQuantity(speedError).InsideOf(-1, 1);
                   if (x.Geo is not null)
                   {
                       var current = x.Geo.VehiclePosition;
                       var heading = x.Geo.VehicleHeading;
                       var target = x.Geo.LookAheadPoint;
                       _operation.SteeringAngle = _steerController.GetSteeringAngle(current, heading, target);
                   }
                   _car.Set(_operation);
               })
               .ToTask();
            Debug.WriteLine("finish following.");
        }

        private async Task TurnAsync(CancellationToken ct)
        {
            Debug.WriteLine("create turn path.");
            var geo = await InfoUpdated.TakeUntil(x => x is not null).Select(x => x.Geo).LastOrDefaultAsync();
            var path = CreateTurnPath(geo.VehiclePosition, geo.VehicleHeading, 4);
            _navigator.InsertPath(_navigator.CurrentPathIndex, path);

            Debug.WriteLine("start turning.");
            await InfoUpdated
               .Where(x => x is not null)
               .TakeUntil(x => ct.IsCancellationRequested)
               .TakeUntil(x => x.Geo.OnThePath && Math.Abs(x.Geo.HeadingError.Degree) < 30)
               .Do(x =>
               {
                   var actualSpeed = x.Vehicle.VehicleSpeed;
                   var speedError = actualSpeed - _targetSpeed;
                   _operation.Throttle += _speedController.GetControlQuantity(speedError).InsideOf(-1, 1);
                   if (x.Geo is not null)
                   {
                       var current = x.Geo.VehiclePosition;
                       var heading = x.Geo.VehicleHeading;
                       var target = x.Geo.LookAheadPoint;
                       _operation.SteeringAngle = _steerController.GetSteeringAngle(current, heading, target);
                   }
                   _car.Set(_operation);
               })
               .ToTask();
            Debug.WriteLine("finish turning.");
        }

        private async Task<bool> IsToTurnAsync()
        {
            var vehicleDirection = (await InfoUpdated.TakeUntil(x => x is not null).LastOrDefaultAsync()).Geo.VehicleHeading;
            var pathDirection = new Vector2D(_navigator.CurrentPath.Points[0], _navigator.CurrentPath.Points[1]).ClockwiseAngleFromY;
            return Math.Abs((vehicleDirection - pathDirection).Degree) > 90;
        }

        private PathData CreateTurnPath(Point2D vehiclePosition, Angle vehicleHeading, double turnRadius)
        {
            var targetPoint = _navigator.CurrentPath.Points[0];
            var desiredDirectionVector = new Vector2D(vehiclePosition, targetPoint).UnitVector;
            var left = desiredDirectionVector.ClockwiseAngleFromY < vehicleHeading;
            var pivot1 = vehiclePosition + desiredDirectionVector * turnRadius;
            var pivot2 = targetPoint - desiredDirectionVector * turnRadius;
            var arc1 = new Arc2D(vehiclePosition, pivot1, new(left ? 90 : -90, AngleType.Degree));
            var arc2 = new Arc2D(pivot2, targetPoint, new(left ? 90 : -90, AngleType.Degree));
            var line = new LineSegment2D(arc1.End, arc2.Start);
            var points = new List<Point2D>();
            points.AddRange(arc1.ApproximateAsPoints(new(10, AngleType.Degree)));
            points.AddRange(line.ApproximateAsPoints(2));
            points.AddRange(arc2.ApproximateAsPoints(new(10, AngleType.Degree)));
            return new(points, "Turn");
        }

    }
}
