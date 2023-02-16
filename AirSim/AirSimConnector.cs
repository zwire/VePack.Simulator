using Husty.IO;
using Husty.Geometry;
using VePack.Connectors.Gnss;
using VePack.Connectors.Imu;
using VePack.Connectors.TextStream;

namespace AirSim;

public class AirSimConnector : TextStreamBase<CarOperation, CarInformation>
{

    // ------ fields ------ //

    private double _steerDeg;


    // ------ constructors ------ //

    public AirSimConnector(IDataTransporter stream) : base(stream, new(10), new(100)) { }


    // ------ protected methods ------ //

    protected override CarInformation ParseToInformation(string msg)
    {
        var t = DateTimeOffset.Now;
        var msgs = msg.Split(",");
        if (msgs.Length > 5 &&
            double.TryParse(msgs[0], out var speed) &&
            double.TryParse(msgs[1], out var lat) &&
            double.TryParse(msgs[2], out var lon) &&
            double.TryParse(msgs[3], out var pitch) &&
            double.TryParse(msgs[4], out var roll) &&
            double.TryParse(msgs[5], out var yaw)
        )
            return new(
                t,
                speed * 3.6,
                Angle.FromDegree(_steerDeg),
                new GnssData(t, 0, lat, lon, GnssQuality.Simulation),
                new ImuData(t, Angle.FromRadian(yaw), Angle.FromRadian(pitch), Angle.FromRadian(roll))
            );
        else
            return null;
    }

    protected override string ParseToOperationText(CarOperation operation)
    {
        _steerDeg = operation.SteeringAngle.Degree;
        return $"{operation.Throttle},{operation.SteeringAngle.Radian},{operation.FootBrake}";
    }

}
