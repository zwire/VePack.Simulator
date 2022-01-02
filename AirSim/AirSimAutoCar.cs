using System;
using System.Diagnostics;
using System.Reactive.Linq;
using System.Threading;
using VePack;
using VePack.CoreModule;
using VePack.Plugin.ControlModule;
using VePack.Plugin.MapModule;

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
            var line = "";
            if (mapFile is not null)
            {
                SetMap(mapFile, plnFile);
                var iniPoint = _navigator.MapData.Paths[0].Points[0];
                foreach (var path in _navigator.MapData.Paths)
                    foreach (var p in path.Points)
                        line += $"{p.Y - iniPoint.Y},{p.X - iniPoint.X},";
            }
            _python = new() { StartInfo = new(pythonExe) { Arguments = $"{pyFile} {line}" } };
            _python.Start();
            _client = new("127.0.0.1", 3000);
            _stream = _client.GetStream();
            _car = new(_stream);
            _speedController = new(PidType.Speed, 0.002, 0, 0.003);
            _steerController = new(2.0);
            _operation = new();
            _car.ConnectSendingStream(TimeSpan.FromMilliseconds(100));
            var observable = _car.ConnectReceivingStream(TimeSpan.FromMilliseconds(100))
                .Finally(() => Dispose())
                .Select(x =>
                {
                    var wgs = new WgsPointData(x.Gnss.Latitude, x.Gnss.Longitude);
                    var position = MapHelper.WgsToUtm(wgs);
                    var heading = x.Imu.Yaw;
                    var lookAheadDistance = Math.Abs(x.VehicleSpeed) + 5;
                    var geoInfo = _navigator?.Update(position, heading, GnssQuality.Simulation, lookAheadDistance);
                    return new CompositeInfo<CarInformation>(
                        _cts is not null && _cts.IsCancellationRequested is false,
                        x, geoInfo
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
        }

        public void Start()
        {
            _cts = new();
            _operation = new();
            InfoUpdated
                .Where(x => x is not null)
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
                .Subscribe(_cts.Token);
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

    }
}
