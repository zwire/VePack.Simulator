using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Mpc;
using VePack.Utilities;
using static System.Math;

namespace AirSim
{

    public class KinematicSteeringModel : DynamicsModelBase, ISteeringDynamicsModel
    {

        // ------ fields ------ //

        private readonly Angle _maxSteering;
        private readonly double _lateralGain;
        private readonly double _headingGain;
        private readonly double _steeringGain;
        private readonly double _controlGain;
        private readonly double _timeConstant;
        private readonly double _lf;
        private readonly double _lr;
        private double _speed;
        private double _curvature;
        private Angle _desiredSteering;


        // ------ properties ------ //

        public override double[] InputMaxConstraints { get; }


        // ------ constructors ------ //

        /// <summary></summary>
        /// <param name="maxSteering">最大舵角(絶対値)</param>
        /// <param name="lateralGain">横方向偏差へのペナルティ</param>
        /// <param name="headingGain">方位偏差へのペナルティ</param>
        /// <param name="steeringGain">舵角へのペナルティ</param>
        /// <param name="controlGain">制御量の変化へのペナルティ</param>
        /// <param name="lf">重心-前輪</param>
        /// <param name="lr">重心-後輪</param>
        /// <param name="timeConstant">タイヤ遅れ時定数</param>
        /// <param name="dt">制御周期</param>
        public KinematicSteeringModel(
            Angle maxSteering,
            double lateralGain,
            double headingGain, 
            double steeringGain, 
            double controlGain,
            double lf, 
            double lr, 
            double timeConstant,
            double dt = 0.1
        ) : base(dt)
        {
            if (
                maxSteering.Degree <= 0 || lateralGain <= 0 || headingGain <= 0 || 
                steeringGain <= 0 || controlGain <= 0 || lf <= 0 || lr <= 0 || timeConstant <= 0
            )
            {
                throw new ArgumentException();
            }
            _maxSteering = maxSteering;
            _lateralGain = lateralGain;
            _headingGain = headingGain;
            _steeringGain = steeringGain;
            _controlGain = controlGain;
            _lf = lf;
            _lr = lr;
            _timeConstant = timeConstant;
            InputMaxConstraints = new[] { _maxSteering.Radian };
            Reset();
        }


        // ------ public methods ------ //

        public void SetSpeed(double speed)
        {
            _speed = speed;
        }

        public void SetCurvature(double curvature)
        {
            _curvature = curvature;
            _desiredSteering = new(Asin(_lr * curvature), AngleType.Radian);
        }


        // ------ override methods ------ //

        public override void Reset()
        {
            _speed = 0;
            _curvature = 0;
            _desiredSteering = Angle.Zero;
        }

        public override Matrix<double> GetGradientOfHamiltonianWithConstraint(
            Matrix<double> predXs,
            Matrix<double> predLams,
            Matrix<double> gxs,
            Matrix<double> us,
            Matrix<double> dummyUs,
            Matrix<double> raws
        )
        {
            var f = new DenseMatrix(us.RowCount, 3);
            for (int i = 0; i < f.RowCount; i++)
            {
                f[i, 0] = _controlGain * (us[i, 0] - _desiredSteering.Radian) + predLams[i, 0] / _timeConstant + 2.0 * raws[i, 0] * us[i, 0];
                f[i, 1] = -0.01 + 2 * raws[i, 0] * dummyUs[i, 0];
                f[i, 2] = Pow(us[i, 0], 2) + Pow(dummyUs[i, 0], 2) - Pow(_maxSteering.Radian, 2);
            }
            return f;
        }

        public override Vector<double> PredictAdjointDot(
            Vector<double> x, 
            Vector<double> gx, 
            Vector<double> u, 
            Vector<double> lams = null
        )
        {
            var lateralError = x[0];
            var headingError = x[1];
            var steering = x[2];
            if (lams is null)
                lams = new DenseVector(x.Count);
            var beta = CalcBeta(steering);
            var dBeta = CalcGradBeta(steering);
            return new DenseVector(new[]
            {
                _lateralGain * lateralError,
                _headingGain * headingError
                    + lams[0] * _speed * Cos(headingError + beta)
                    + lams[1] * _curvature * _speed * Sin(headingError + beta),
                _steeringGain * (steering - _desiredSteering.Radian)
                    + lams[1] * _speed * Cos(beta) * dBeta / _lr
                    - lams[2] / _timeConstant
                    + lams[0] * _speed * Cos(headingError + beta) * dBeta
                    + lams[1] * _curvature * _speed * Sin(headingError + beta) * dBeta
            });
        }

        public override Vector<double> PredictGradientOfState(Vector<double> x, Vector<double> u)
        {
            var lateralError = x[0];
            var headingError = x[1];
            var steering = x[2];
            var beta = CalcBeta(steering);
            return new DenseVector(new[]
            {
                _speed * Sin(headingError + beta),
                _speed * Sin(beta) / _lr
                    - _curvature * _speed * Cos(headingError + beta),
                (steering - _desiredSteering.Radian) / _timeConstant
            });
        }


        // ------ private methods ------ //

        private double CalcBeta(double steering)
        {
            return Atan2(_lr * Tan(steering), _lf + _lr);
        }

        private double CalcGradBeta(double steering)
        {
            var ll = _lr / (_lf + _lr);
            return ll / Pow(Cos(steering), 2) / (1 + Pow(ll * Tan(steering), 2));
        }

    }
}
