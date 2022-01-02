using System;
using VePack.CoreModule;

namespace AirSim
{
    public record CarInformation(
        DateTimeOffset Time,
        double VehicleSpeed,
        Angle SteeringAngle,
        GnssData Gnss,
        ImuData Imu
    );

    public class CarOperation
    {

        public double Throttle { get; set; } = 0;

        public Angle SteeringAngle { get; set; } = new(0, AngleType.Degree);

        public double FootBrake { get; set; } = 0;

    }
}
