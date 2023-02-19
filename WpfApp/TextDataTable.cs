using AirSim;
using VePack;

namespace WpfApp;

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
            LateralErrorValue = $"{info.Guidance.LateralError:f3} m",
            HeadingErrorValue = $"{info.Guidance.HeadingError.Degree:f1} deg",
            GnssQualityValue = $"{info.Gnss.GnssQuality}",
            LatitudeValue = $"{info.Gnss.Latitude:f10}",
            LongitudeValue = $"{info.Gnss.Longitude:f10}",
            YawValue = $"{info.Imu.Yaw.Degree:f1} deg",
            PitchValue = $"{info.Imu.Pitch.Degree:f1} deg",
            RollValue = $"{info.Imu.Roll.Degree:f1} deg"
        };
}
