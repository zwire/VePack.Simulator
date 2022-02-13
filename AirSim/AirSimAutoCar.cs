using System;
using System.Linq;
using System.Diagnostics;
using System.Collections.Generic;
using System.Reactive.Linq;
using System.Reactive.Threading.Tasks;
using System.Threading;
using System.Threading.Tasks;
using VePack;
using VePack.Utilities;
using VePack.Plugin.Navigation;
using VePack.Plugin.Controllers;

namespace AirSim
{
    public class AirSimAutoCar : IAutoVehicle<CarInformation>
    {

        // ------ fields ------ //

        private const string pythonExe = "..\\..\\..\\..\\..\\..\\..\\AppData\\Local\\Programs\\Python\\Python37\\python.exe";
        private const string pyFile = "..\\..\\..\\..\\AirSim\\airsim_server.py";
        private const double _turnRadius = 6;
        private readonly AirSimConnector _car;
        private readonly Process _python;
        private readonly TcpSocketClient _client;
        private readonly BidirectionalDataStream _stream;
        private readonly Pid _speedController;
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
            _speedController = new(PidType.Speed, 0.002, 0, 0.003);
            _steerController = new(
                new(1.0, 1.0, 1.0),
                new[] { 5, 8, 10 }, 
                0.7,
                new(35, AngleType.Degree),
                new(5, AngleType.Degree)
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
            var map = NaviHelper.LoadMapFromFile(mapFile);
            if (plnFile is not null)
                map = NaviHelper.ModifyMapByPlnFile(plnFile, map);
            _navigator = new(map, 1e-3);
            _navigator.CurrentPathChanged.Finally(() => Dispose());
            var paths = _navigator.MapData.Clone().Paths;
            for (int i = 0; i < paths.Count - 2; i++)
            {
                if (i % 2 is 1)
                    paths[i] = paths[i].GetReverse();
                var turnPath = GenerateTurnPath(paths[i], paths[i + 1]);
                _navigator.InsertPath(2 * i + 1, turnPath);
            }
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

        private UtmPathData GenerateTurnPath(UtmPathData passedPath, UtmPathData targetPath)
        {
            targetPath = NaviHelper.ModifyPathDirection(targetPath, passedPath.Points[^1]);
            var entranceDirectionVector = new Vector2D(targetPath.Points[0], targetPath.Points[1]).UnitVector;
            var desiredDirectionVector = new Vector2D(passedPath.Points[^1], targetPath.Points[0]).UnitVector;
            var end = targetPath.Points[0] - entranceDirectionVector * 2;
            var left = entranceDirectionVector.GetClockwiseAngleFrom(desiredDirectionVector) < Angle.Zero;
            var d = passedPath.Points[^1].DistanceTo(targetPath.Points[0]);

            if (passedPath.Points[^1].DistanceTo(targetPath.Points[0]) > _turnRadius * 2)
            {
                var arc2 = new Arc2D(end, end - desiredDirectionVector * _turnRadius, 
                    new(left ? -90 : 90, AngleType.Degree)).GetReverse().ApproximateAsPoints(new(15, AngleType.Degree));
                var start = passedPath.Points[^1] - entranceDirectionVector * 2;
                var arc1 = new Arc2D(start, start + desiredDirectionVector * _turnRadius, 
                    new(left ? 90 : -90, AngleType.Degree)).ApproximateAsPoints(new(15, AngleType.Degree));
                var list = new List<UtmPointData>();
                list.AddRange(arc1.Select(a => new UtmPointData(a)));
                list.AddRange(arc2.Select(a => new UtmPointData(a)));
                list.Add(new(list.LastOrDefault() + entranceDirectionVector * 0.1));
                return new(list, left ? "Turn Left" : "Turn Right");
            }
            else
            {
                var arc = new Arc2D(end, end - desiredDirectionVector * _turnRadius,
                    new(left ? -180 : 180, AngleType.Degree)).GetReverse().ApproximateAsPoints(new(15, AngleType.Degree));
                var list = new List<UtmPointData>();
                list.AddRange(arc.Select(a => new UtmPointData(a)));
                list.Add(new(list.LastOrDefault() + entranceDirectionVector * 0.1));
                return new(list, left ? "Turn Left" : "Turn Right");
            }
        }

        private async Task FollowAsync(CancellationToken ct)
        {
            Console.WriteLine($"\nstart to follow path {_navigator.CurrentPathIndex}.\n");
            var curvature = 0.0;
            if ((string)_navigator.CurrentPath.Id is "Turn Left")
                curvature = -1.0 / _turnRadius;
            if ((string)_navigator.CurrentPath.Id is "Turn Right")
                curvature = 1.0 / _turnRadius;
            await InfoUpdated
                .Where(x => x?.Geo is not null && x?.Vehicle is not null)
                .TakeUntil(x => ct.IsCancellationRequested)
                .TakeUntil(_navigator.CurrentPathChanged)
                .Where(_ => _autoSteering)
                .Do(x =>
                {
                    _steerController.SetParams(x.Vehicle.VehicleSpeed / 3.6, curvature);
                    var angle = _steerController.GetSteeringAngle(
                        x.Geo.LateralError, x.Geo.HeadingError, x.Vehicle.SteeringAngle
                    );
                    SetSteeringAngle(angle);
                    Console.Write($"Steer: {angle.Degree:f1} ... ");
                })
                .ToTask();
        }

    }
}
