using System;
using VePack.Utilities.Geometry;
using VePack.Connectors.Gnss;
using VePack.Connectors.Imu;

namespace AirSim
{
    public record CarInformation(
        DateTimeOffset Time,
        double VehicleSpeed,
        Angle SteeringAngle,
        GnssData Gnss,
        ImuData Imu
    )
    {
        public CarInformation() : this(DateTimeOffset.Now, default, default, default, default) { }
    }

    public class CarOperation
    {

        public double Throttle { get; set; } = 0;

        public Angle SteeringAngle { get; set; } = Angle.Zero;

        public double FootBrake { get; set; } = 0;

    }


    public class Rootobject
    {
        public string LogFile { get; set; }
        public string PythonExe { get; set; }
        public string PyFile { get; set; }
        public string MapFile { get; set; }
        public string PlnFile { get; set; }
        public bool AutoSteering { get; set; }
        public float PathEndMargin { get; set; }
        public bool AutoDirectionModification { get; set; }
        public float TurnRadius { get; set; }
        public Steeringcontrollerparams SteeringControllerParams { get; set; }
        public string SteeringModelFile { get; set; }
        public bool TrainModel { get; set; }
    }

    public class Steeringcontrollerparams
    {
        public int[] PfcCoincidenceIndexes { get; set; }
        public float PfcControlGain { get; set; }
    }

}
