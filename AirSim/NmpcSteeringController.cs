using Mpc;
using VePack.Utilities;

namespace AirSim
{
    public class NmpcSteeringController
    {

        // ------ fields ------ //

        private readonly ISteeringDynamicsModel _model;
        private readonly Nmpc _nmpc;


        // ------ constructors ------ //

        public NmpcSteeringController(
            ISteeringDynamicsModel model,
            int predictionLength = 10,
            double tolerance = 0.01,
            double zeta = 10,
            double dt = 0.1
        )
        {
            _model = model;
            _nmpc = new(model, 1, predictionLength, tolerance, zeta, dt);
        }


        // ------ public methods ------ //

        public Angle GetSteeringAngle(
            double lateralError,
            Angle headingError,
            Angle steering,
            double vehicleSpeed,
            double curvature = default
        )
        {
            if (vehicleSpeed < 0.5) return Angle.Zero;
            _model.SetSpeed(vehicleSpeed);
            _model.SetCurvature(curvature);
            var state = new[] { lateralError, headingError.Radian, steering.Radian };
            var output = _nmpc.GetControlQuantity(state);
            return new(output[0], AngleType.Radian);
        }

        public void Reset()
        {
            _model.Reset();
            _nmpc.Reset();
        }

    }
}
