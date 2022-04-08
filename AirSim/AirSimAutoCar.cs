using System;
using System.Linq;
using System.Diagnostics;
using System.Collections.Generic;
using System.IO;
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
using VePack.Utilities.Cmac;
using VePack.Plugin.Navigation;
using VePack.Plugin.Controllers.ModelFree;
using VePack.Plugin.Controllers.ModelBased.Steering;

namespace AirSim
{
    public sealed class AirSimAutoCar : IAutoVehicle<CarInformation>
    {

        #region fields

        private readonly Rootobject _config;
        private readonly AirSimConnector _car;
        private readonly Process _python;
        private readonly TcpSocketClient _client;
        private readonly BidirectionalDataStream _stream;
        private readonly Pid _speedController;
        private readonly NnSteeringModel _steerModel;
        private readonly PfcSteeringController _steerController;
        private readonly IDisposable _connector;
        private readonly bool _autoSteering;
        private readonly StreamWriter _sw;
        private double _targetSpeed;
        private MapNavigator _navigator;
        private CarOperation _operation;
        private CancellationTokenSource _cts;

        #endregion fields


        #region properties

        public IObservable<CompositeInfo<CarInformation>> InfoUpdated { get; }

        #endregion properties


        #region constructors

        public AirSimAutoCar()
        {
            _config = new ConfigurationBuilder().AddJsonFile("rootsettings.json").Build().Get<Rootobject>();
            _sw = (_config.LogFile is not null && _config.LogFile.Contains(".csv")) ? new(_config.LogFile) : null;
            _sw?.WriteLine("x__0:lateral,x__1:heading,x__2:steer,x__3:speed,x__4:curvature,y__0:lateral,y__1:heading");
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
            //_steerModel = new KinematicSteeringModel(1.5, 1.5, 0.1);
            _steerModel = new NnSteeringModel(
                NetworkGraph.Load(_config.SteeringModelFile),
                _config.TrainModel ? 4 : 0, 
                0.1
            );
            //_steerModel = new CmacSteeringModel(
            //    CmacBundler.Load(_config.SteeringModelFile),
            //    _config.TrainModel,
            //    1.5, 1.5, 0.1
            //);
            var s = _config.SteeringControllerParams;
            _steerController = new(
                _steerModel,
                s.PfcCoincidenceIndexes,
                s.PfcControlGain,
                new(35, AngleType.Degree),
                new(10, AngleType.Degree)
            );

            var line = "";
            if (_config.MapFile is not null && _config.MapFile is not "")
            {
                SetMap(_config.MapFile, _config.PlnFile);
                var iniPoint = _navigator.MapData.Paths[0][0];
                foreach (var path in _navigator.MapData.Paths)
                    foreach (var p in path)
                        line += $"{p.Y - iniPoint.Y},{p.X - iniPoint.X},";
            }
            _stream.WriteString(line);
            _car.ConnectSendingStream(new Freq(100));

            double? preLat = null;
            double? preHead = null;
            double? preSteer = null;
            double? preSpeed = null;
            double? preCurva = null;
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
                        var lateral = x.Geo.LateralError;
                        var heading = x.Geo.HeadingError;
                        var steer = x.Vehicle.SteeringAngle;
                        var speed = x.Vehicle.VehicleSpeed / 3.6;
                        var pathCurvature = 0.0;
                        double.TryParse((string)_navigator.CurrentPath.Id, out pathCurvature);
                        if (speed <= 0 || pathCurvature == 0)
                        {
                            preLat = null;
                            preHead = null;
                            preSpeed = null;
                            preSteer = null;
                            preCurva = null;
                        }
                        if (preLat is not null && preCurva != 0)
                            _sw?.WriteLine($"{preLat},{preHead},{preSteer},{preSpeed},{preCurva},{lateral},{heading.Radian}");
                        preLat = lateral;
                        preHead = heading.Radian;
                        preSpeed = speed;
                        preSteer = steer.Radian;
                        preCurva = pathCurvature;
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

        #endregion constructors


        #region public methods

        public void Dispose()
        {
            Stop();
            _sw?.Close();
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
                await FollowAsync(_cts.Token);
                if (await IsToTurnAsync())
                    await TurnAsync();
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

        public void SetSteeringAngle(Angle angle)
        {
            _operation.SteeringAngle = new(angle.Degree.InsideOf(-35, 35), AngleType.Degree);
            _car?.Set(_operation);
        }

        #endregion public methods


        #region private methods

        private void SetBrake(int level)
        {
            _operation.Throttle = 0;
            _operation.FootBrake = level;
            _car.Set(_operation);
        }

        private (UtmPathData, UtmPathData) GenerateTurnPath(UtmPathData passedPath, UtmPathData targetPath)
        {
            // 次パスの向きを修正
            targetPath = targetPath.ModifyPathDirection(passedPath[^1]);
            // 次パスの入口周辺の位置と座標系をとる
            var dx = targetPath.GetSegment(0).ToVector2D().UnitVector;
            var start = passedPath[^1] - dx * _config.PathEndMargin;
            var end = targetPath[0] - dx * _config.PathEndMargin;
            var edgeVector = new Vector2D(start, end).UnitVector;
            var left = start.IsOnTheLeft(targetPath.GetSegment(0));
            var dy = dx.Rotate(new(left ? -90 : 90, AngleType.Degree));

            // パス端のズレ角度
            var slippage = edgeVector.GetClockwiseAngleFrom(dy);
            if (left)
                slippage = -slippage;
            // 前後半の旋回半径
            var r1 = 1 / (1 + Math.Sin(slippage.Radian));
            var r2 = 1 / (1 - Math.Sin(slippage.Radian));
            // 小さい方が指定した半径になるように揃える
            var scale = _config.TurnRadius / Math.Min(r1, r2);
            r1 *= scale;
            r2 *= scale;
            // 前後半の旋回中心
            var o1 = start + r1 * dy;
            var o2 = end - r2 * dy;

            // このままだと無駄に大きい円ができるので、大きい円を縮める
            var e1 = new Arc2D(start, o1, new((90 + slippage.Degree) * (left ? 1 : -1), AngleType.Degree)).End;
            var e2 = new Arc2D(end, o2, new((90 - slippage.Degree) * (left ? -1 : 1), AngleType.Degree)).End;
            var headlandLine = new Line2D(e1, e2);
            var hypotenuseLength = _config.TurnRadius / Math.Cos(new Angle((90 - slippage.Degree) / 2, AngleType.Degree).Radian);
            if (r1 < r2)
            {
                var targetLine = targetPath.GetSegment(0);
                var intersection = headlandLine.GetIntersection(targetLine);
                var direction = new Vector2D(intersection, o2).UnitVector;
                o2 = intersection + direction * hypotenuseLength;
                end = targetLine.GetPerpendicularFoot(o2);
                r2 = r1;
            }
            else
            {
                var targetLine = passedPath.GetSegment(-1);
                var intersection = headlandLine.GetIntersection(targetLine);
                var direction = new Vector2D(intersection, o1).UnitVector;
                o1 = intersection + direction * hypotenuseLength;
                start = targetLine.GetPerpendicularFoot(o1);
                r1 = r2;
            }

            // 弧を得る
            var arc1 = new Arc2D(start, o1, new((90 + slippage.Degree) * (left ? 1 : -1), AngleType.Degree)).ApproximateAsPoints(new(5, AngleType.Degree));
            var arc2 = new Arc2D(end, o2, new((90 - slippage.Degree) * (left ? -1 : 1), AngleType.Degree)).GetReverse().ApproximateAsPoints(new(5, AngleType.Degree));

            // 端におめかししてリスト化
            var list1 = new List<UtmPointData>();
            list1.Add(new(arc1[0] + dx * 0.1));
            list1.AddRange(arc1.Select(a => new UtmPointData(a)));
            list1.Add(new(arc1[^1] + edgeVector * 0.1));
            var list2 = new List<UtmPointData>();
            list2.Add(new(arc2[0] - edgeVector * 0.1));
            list2.AddRange(arc2.Select(a => new UtmPointData(a)));
            list2.Add(new(arc2[^1] + dx * 0.1));

            return new(new(list1, $"{(left ? -1 : 1) / r1}"), new(list2, $"{(left ? -1 : 1) / r2}"));
        }

        private async Task<CompositeInfo<CarInformation>> FollowAsync(CancellationToken ct)
        {
            Console.WriteLine($"\nstart to follow path {_navigator.CurrentPathIndex}.\n");
            var pathCurvature = 0.0;
            double.TryParse((string)_navigator.CurrentPath.Id, out pathCurvature);
            if (pathCurvature is 0)
                _navigator.CurrentPath.ExtendLast(2);
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
                    var curvature = x.Geo.OnThePath ? -pathCurvature : 0;
                    //_steerModel.SetParams(speed, curvature);
                    _steerModel.PredictDynamics(lateral, heading, steer, speed, curvature);
                    _steerController.CalcParams();
                    var angle = _steerController.GetSteeringAngle(lateral, heading, steer);
                    SetSteeringAngle(angle);
                    Console.Write($"Steer: {angle.Degree:f1} ... ");
                })
                //.Finally(() => _steerModel.SaveModel("latest.ydn"))
                .ToTask();
        }

        private async Task<bool> IsToTurnAsync()
        {
            return Math.Abs((await InfoUpdated.FirstOrDefaultAsync()).Geo.HeadingError.Degree) > 90;
        }

        private async Task TurnAsync()
        {
            // 先にパスを得る
            var (first, second) = GenerateTurnPath(_navigator.MapData.Paths[_navigator.CurrentPathIndex - 1], _navigator.CurrentPath);
            // 前半パスを追従
            _navigator.InsertPath(_navigator.CurrentPathIndex, first);
            var ckpt = await FollowAsync(_cts.Token);
            var seg = second.GetSegment(0);
            // バックが必要かの判断
            if (ckpt.Geo.VehiclePosition.IsInFrontOf(seg))
            {
                SetSteeringAngle(Angle.Zero);
                _steerController.Reset();
                SetBrake(2);
                await Task.Delay(200);
                SetVehicleSpeed(-_targetSpeed);
                // secondの始点前までバック
                await InfoUpdated.TakeWhile(x => x.Geo.VehiclePosition.IsInFrontOf(seg)).ToTask();
                SetBrake(2);
                await Task.Delay(200);
                SetVehicleSpeed(-_targetSpeed);
            }
            // 後半パスを追従
            _navigator.InsertPath(_navigator.CurrentPathIndex, second);
            await FollowAsync(_cts.Token);
        }

        #endregion private methods

    }
}
