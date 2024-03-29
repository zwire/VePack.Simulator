﻿using System.Diagnostics;
using System.Reactive.Linq;
using System.Reactive.Threading.Tasks;
using Microsoft.Extensions.Configuration;
using Husty;
using Husty.Extensions;
using Husty.Geometry;
using Husty.Communication;
using Husty.NeuralNetwork;
using Husty.NeuralNetwork.Cmac;
using VePack;
using VePack.Connectors.Imu;
using VePack.Controllers.ModelFree;
using VePack.Controllers.ModelBased.Steering;
using VePack.Guidance;

namespace AirSim;

public sealed class AirSimAutoCar : IVehicle<CarInformation>
{

    // ------ fields ------ //

    private readonly Rootobject _config;
    private readonly AirSimConnector _car;
    private readonly Process _python;
    private readonly TcpSocketClient _client;
    private readonly IDataTransporter _stream;
    private readonly IDisposable _connector;
    private readonly Pid _speedController;
    private readonly KinematicSteeringModel _steerModel;
    private readonly ISteeringController _steerController;
    private readonly LsmHeadingCorrector _imuFilter;
    private readonly DataLogger<CompositeInfo<CarInformation>> _logger;
    private double _targetSpeed;
    private MapGuide _guide;
    private CarOperation _operation;
    private CancellationTokenSource _cts;

    private readonly double _actuatorOffsetLength = 0.0;


    // ------ properties ------ //

    public IObservable<CompositeInfo<CarInformation>> InfoUpdated { get; }


    // ------ constructors ------ //

    public AirSimAutoCar()
    {
        _config = new ConfigurationBuilder().AddJsonFile("rootsettings.json").Build().Get<Rootobject>();
        _logger = _config.LogFile is "" 
            ? null 
            : new(Path.GetExtension(_config.LogFile).Contains("csv") 
                ? LogFileFormat.Csv 
                : LogFileFormat.Json, 
            _config.LogFile
        );
        _cts = new();
        _cts.Cancel();
        _python = new() { StartInfo = new(_config.PythonExe) { Arguments = _config.PyFile} };
        _python.Start();
        _client = new("127.0.0.1", 3000);
        _stream = _client.GetStream().Value;
        _car = new(_stream);
        _operation = new();
        _imuFilter = new();
        _speedController = new(PidType.Speed, 0.001, 0, 0.003);
        _steerModel = _config.UseNN 
            ? new NnSteeringModel(
                _config.SteeringModelFile.EndsWith(".ynn")
                    ? NetworkGraph.Load(_config.SteeringModelFile)
                    : CmacBundler.Load(_config.SteeringModelFile), 
                false)
            : new KinematicSteeringModel(1.6, 0.4);

        _steerController = new PfcSteeringController(
            _steerModel,
            _config.PfcCoincidenceIndexes,
            (i, v) =>
            {
                var convergenceTime = 3.0;  // 参照軌道とパスが収束する時間(s)
                var sharpness = 0.2;        // S字の曲がり具合
                var speed = _steerModel.VehicleSpeed;
                var lateralE = v[0];
                var headingE = Angle.FromRadian(v[1]);
                var convergenceDistance = convergenceTime * Math.Abs(speed);
                var margin = sharpness * convergenceDistance / 2;
                var targetPosition = i * _steerModel.Dt / convergenceTime;
                var switchBack = _guide.CurrentPath.Id is "Back" || _guide.NextPath?.Id is "Back";
                Pose2D tp = null;
                if (targetPosition > 1)
                {
                    tp = _guide.GetLookAheadPointFromReferencePoint(convergenceDistance * targetPosition, !switchBack);
                }
                else
                {
                    var startPoint = new Pose2D(new(lateralE, 0), headingE);
                    var endPoint = _guide.GetLookAheadPointFromReferencePoint(convergenceDistance, !switchBack);
                    tp = GuidanceHelper.GetTrajectoryPointFromBezierCurve(startPoint, endPoint, targetPosition, margin);
                }
                if (speed < 0) tp = new(new(-tp.Position.X, -tp.Position.Y), tp.Heading);
                return new double[] { tp.Position.X, tp.Heading.Radian, 0, 0 };
            },
            1.0,
            Angle.FromDegree(35),
            Angle.FromDegree(10)
        );

        var line = "";
        if (_config.MapFile is not null && _config.MapFile is not "")
        {
            var map = GuidanceHelper.LoadMapFromFile(_config.MapFile);
            // 先に旋回パスも作っとく
            var stride = 1;
            for (int i = 0; i < map.Paths.Count - 1; i += stride)
            {
                if (map.Paths[i].Id is "Work" && _config.PathEndMargin > 0)
                    map.Paths[i].ExtendLast(_config.PathEndMargin);
                map.Paths[i + 1] = GuidanceHelper.ModifyPathDirection(map.Paths[i + 1], map.Paths[i].Points[^1]);
                var paths = GuidanceHelper.GenerateTurnPath(map.Paths[i], map.Paths[i + 1], _config.TurnRadius, 1);
                stride = 1;
                map.Paths.Insert(i + stride++, paths.FirstHalf);
                map.Paths.Insert(i + stride++, paths.Bridge);
                map.Paths.Insert(i + stride++, paths.SecondHalf);
            }
            _guide = new(map) { AutoDirectionModification = false };
            _guide.CurrentPathChanged.Finally(Dispose);
            var iniPoint = _guide.MapData.Paths[0].Points[0];
            foreach (var path in _guide.MapData.Paths)
                foreach (var p in path.Points)
                    line += $"{p.Y - iniPoint.Y},{p.X - iniPoint.X},";
        }
        _stream.TryWriteLineAsync(line).Wait();

        var observable = _car.ReceivingStream
            .Finally(Dispose)
            .Select(d =>
            {
                var (x, y, _) = new WgsPointData(d.Gnss.Latitude, d.Gnss.Longitude).ToUtm();
                var heading = _imuFilter.Correct(d.Imu.Yaw, new(x, y), d.SteeringAngle, d.VehicleSpeed / 3.6);
                x -= (float)(_config.AntennaOffset * Math.Sin(heading.Radian));
                y -= (float)(_config.AntennaOffset * Math.Cos(heading.Radian));
                x += (float)(_actuatorOffsetLength * Math.Sin(heading.Radian));
                y += (float)(_actuatorOffsetLength * Math.Cos(heading.Radian));
                var imuData = new ImuData(DateTimeOffset.Now, heading);
                return new CompositeInfo<CarInformation>(
                    _cts is not null && _cts.IsCancellationRequested is false,
                    d, d.Gnss, d.Imu, _guide?.Update(new(x, y), heading)
                );
            })
            .TakeWhile(x =>
            {
                if (x?.Guidance is not null)
                {
                    _logger?.Write(x);
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
        _logger?.Dispose();
        _connector.Dispose();
        _car.Dispose();
        _stream.Dispose();
        _client.Dispose();
        _python.Dispose();
    }

    public async void Start()
    {
        _cts = new();
        _operation = new();
        _car.Set(_operation);

        InfoUpdated
           .Where(x => x?.Guidance is not null && x?.Vehicle is not null)
           .Where(_ => _operation.FootBrake is 0)
           .TakeUntil(x => _cts.IsCancellationRequested)
           .Do(x =>
           {
               var speedError = x.Vehicle.VehicleSpeed - _targetSpeed;
               _operation.Throttle += _speedController.GetControlQuantity(speedError).InsideOf(-1, 1);
               _car.Set(_operation);
           })
           .Subscribe();

        while (_config.AutoSteering && !_cts.IsCancellationRequested)
            await FollowAsync(_cts.Token);

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

        Console.WriteLine($"\nstart to follow path {_guide.CurrentPathIndex}.\n");
        await InfoUpdated.TakeUntil(x => Math.Abs(x.Vehicle.VehicleSpeed) > 0.1).ToTask();

        if (_guide.CurrentPath.Id is "Back")
        {
            SetSteeringAngle(Angle.Zero);
            SetBrake(2);
            await Task.Delay(200, ct);
            SetVehicleSpeed(-_targetSpeed);
            await InfoUpdated
                .Where(x => x?.Guidance is not null && x?.Vehicle is not null)
                .TakeWhile(x => !ct.IsCancellationRequested)
                .TakeUntil(_guide.CurrentPathChanged)
                .Do(x =>
                {
                    var lateral = -x.Guidance.LateralError;
                    var heading = x.Guidance.HeadingError - Angle.FromRadian(Math.PI);
                    var steer = x.Vehicle.SteeringAngle;
                    var speed = x.Vehicle.VehicleSpeed / 3.6;
                    double.TryParse(_guide.CurrentPoint.Id, out var curvature);
                    _steerModel.UpdateA(lateral, heading, steer, speed, curvature);
                    _steerModel.A[0, 2] += _steerModel.A[1, 2] * _actuatorOffsetLength;
                    var angle = _steerController.GetSteeringAngle(lateral, heading, steer);
                    SetSteeringAngle(angle);
                    Console.Write($"Steer: {angle.Degree:f1} ... ");
                })
                .ToTask();
            await Task.Delay(200);
            SetBrake(2);
            await Task.Delay(200, ct);
            SetVehicleSpeed(-_targetSpeed);
        }

        return await InfoUpdated
            .Where(x => x?.Guidance is not null && x?.Vehicle is not null)
            .TakeWhile(x => !ct.IsCancellationRequested)
            .TakeUntil(_guide.CurrentPathChanged)
            .Do(x =>
            {
                var lateral = x.Guidance.LateralError;
                var heading = x.Guidance.HeadingError;
                var steer = x.Vehicle.SteeringAngle;
                var speed = x.Vehicle.VehicleSpeed / 3.6;
                double.TryParse(_guide.CurrentPoint.Id, out var curvature);
                _steerModel.UpdateA(lateral, heading, steer, speed, curvature);
                _steerModel.A[0, 2] += _steerModel.A[1, 2] * _actuatorOffsetLength;
                var angle = _steerController.GetSteeringAngle(lateral, heading, steer);
                SetSteeringAngle(angle);
                Console.Write($"Steer: {angle.Degree:f1} ... ");
            })
            .ToTask();

    }

}
