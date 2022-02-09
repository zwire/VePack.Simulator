using VePack.Utilities;
using VePack.Plugin.Controllers;

namespace AirSim
{
    public class PfcSteeringController
    {

        // ------ fields ------ //

        private readonly Pfc _pfc;
        private readonly PfcSteeringModel _model;


        // ------ constructors ------ //

        public PfcSteeringController(
            PfcSteeringModel model,
            Angle maxSteering,
            Angle maxSteeringSpeed,
            double controlGain,
            int[] coincidenceIndexes
        )
        {
            _model = model;
            _pfc = new(
                model.A, model.B, 
                1.0 / controlGain, coincidenceIndexes, 
                new[] { maxSteering.Radian }, new[] { maxSteeringSpeed.Radian }, 
                model.Dt
            );
        }


        // ------ public methods ------ //

        public Angle GetSteeringAngle(
            double lateralError,
            Angle headingError,
            Angle steeringAngle,
            double vehicleSpeed,
            double curvature = 0
        )
        {
            if (vehicleSpeed < 0.1) return Angle.Zero;
            _model.Update(vehicleSpeed, curvature);
            var e = new[] { lateralError * 1, headingError.Radian * 1, steeringAngle.Radian * 1 };
            _pfc.SetParams(_model.A, _model.B);
            var quantity = _pfc.GetControlQuantity(e, _model.W.ToColumnMajorArray());
            return new(quantity[0], AngleType.Radian);
        }

        public void Reset()
        {
            _model.Reset();
        }

    }
}
