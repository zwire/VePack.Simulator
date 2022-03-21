using System;
using VePack.Utilities;
using VePack.Connectors;

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

        public Angle SteeringAngle { get; set; } = Angle.Zero;

        public double FootBrake { get; set; } = 0;

    }


    public class Rootobject
    {
        public string PythonExe { get; set; }
        public string PyFile { get; set; }
        public string MapFile { get; set; }
        public string PlnFile { get; set; }
        public bool AutoSteering { get; set; }
        public float PathEndMargin { get; set; }
        public bool AutoDirectionModification { get; set; }
        public float TurnRadius { get; set; }
        public Vehiclemodelparams VehicleModelParams { get; set; }
        public Steeringcontrollerparams SteeringControllerParams { get; set; }
        public string NNSteeringModelFile { get; set; }
        public int NNTrainBatchSize { get; set; }
    }

    public class Vehiclemodelparams
    {
        public float Lf { get; set; }
        public float Lr { get; set; }
        public float TimeConstant { get; set; }
    }

    public class Steeringcontrollerparams
    {
        public int[] PfcCoincidenceIndexes { get; set; }
        public float PfcControlGain { get; set; }
    }

}
