using AirSim;
using VePack;

namespace WpfApp
{
    public class TextDataTable
    {

        public string VehicleSpeedValue { private set; get; } = "";
        public string SteeringAngleValue { private set; get; } = "";
        public string LateralErrorValue { private set; get; } = "";
        public string HeadingErrorValue { private set; get; } = "";
        public string GnssQualityValue { private set; get; } = "";
        public string LatitudeValue { private set; get; } = "";
        public string LongitudeValue { private set; get; } = "";
        public string YawValue { private set; get; } = "";
        public string PitchValue { private set; get; } = "";
        public string RollValue { private set; get; } = "";

        public static TextDataTable FromCompositeInfo(CompositeInfo<CarInformation> info)
            => new()
            {
                VehicleSpeedValue = $"{info.Vehicle.VehicleSpeed:f1} km/h",
                SteeringAngleValue = $"{info.Vehicle.SteeringAngle.Degree:f1} deg",
                LateralErrorValue = $"{info.Geo.LateralError:f3} m",
                HeadingErrorValue = $"{info.Geo.HeadingError.Degree:f1} deg",
                GnssQualityValue = $"{info.Geo.GnssQuality}",
                LatitudeValue = $"{info.Vehicle.Gnss.Latitude:f10}",
                LongitudeValue = $"{info.Vehicle.Gnss.Longitude:f10}",
                YawValue = $"{info.Vehicle.Imu.Yaw.Degree:f1} deg",
                PitchValue = $"{info.Vehicle.Imu.Pitch.Degree:f1} deg",
                RollValue = $"{info.Vehicle.Imu.Roll.Degree:f1} deg"
            };
    }
}
