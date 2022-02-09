using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using VePack.Utilities;
using static System.Math;

namespace AirSim
{
    public class PfcSteeringModel
    {
        // ------ fields ------ //

        private readonly double _wheelBase;
        private readonly double _dt;
        private Angle _desiredSteering;


        // ------ properties ------ //

        public double Dt => _dt;

        public Matrix<double> A { private set; get; }

        public Matrix<double> B { private set; get; }

        public Matrix<double> W { private set; get; }


        // ------ constructors ------ //

        /// <summary></summary>
        /// <param name="maxSteering">最大舵角(絶対値)</param>
        /// <param name="lateralGain">横方向偏差へのペナルティ</param>
        /// <param name="headingGain">方位偏差へのペナルティ</param>
        /// <param name="steeringGain">舵角へのペナルティ</param>
        /// <param name="controlGain">舵角速度へのペナルティ</param>
        /// <param name="lf">重心-前輪</param>
        /// <param name="lr">重心-後輪</param>
        /// <param name="timeConstant">タイヤ遅れ時定数</param>
        public PfcSteeringModel(
            double wheelBase,
            double timeConstant,
            double dt = 0.1
        )
        {
            if (wheelBase <= 0 || timeConstant <= 0)
            {
                throw new ArgumentException();
            }
            _wheelBase = wheelBase;
            _dt = dt;
            A = new DenseMatrix(3, 3, new double[]
            {
                1, 0, 0,
                0, 1, 0,
                0, 0, 1.0 - dt / timeConstant
            }).Transpose();
            B = new DenseMatrix(3, 1, new double[]
            {
                0, 0, dt / timeConstant
            });
            W = new DenseMatrix(3, 1);
            Reset();
        }


        // ------ public methods ------ //

        public void Update(double speed, double curvature)
        {
            _desiredSteering = new(Asin(_wheelBase * curvature), AngleType.Radian);
            A[0, 1] = speed * _dt;
            A[1, 2] = speed * _dt / (_wheelBase * Pow(Cos(_desiredSteering.Radian), 2));
            W[1, 0] = -speed * _desiredSteering.Radian * _dt / (_wheelBase * Pow(Cos(_desiredSteering.Radian), 2));
        }

        public void Reset()
        {
            _desiredSteering = Angle.Zero;
        }

    }
}
