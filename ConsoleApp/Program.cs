using AirSim;
using VePack.Utilities.Geometry;

namespace ConsoleApp;

internal class Program
{
    static void Main(string[] args)
    {

        using var sw = new StreamWriter("log_.csv");
        sw.WriteLine("lateral,heading,steer,speed,curvature");
        
        var car = new AirSimAutoCar();
        car.Start();
        car.InfoUpdated.Subscribe(x =>
        {
            var lateral = x.Geo.LateralError;
            var heading = x.Geo.HeadingError;
            var steer = x.Vehicle.SteeringAngle;
            var speed = x.Vehicle.VehicleSpeed;
            sw.WriteLine($"{lateral},{heading.Radian},{steer.Radian},{speed / 3.6},{0}");
            Console.WriteLine($"Speed: {speed:f1}km/h, E: {lateral:f3}m, {heading.Degree:f1}deg");
        });

        var targetSpeed = 0.0;
        var targetAngle = 0.0;
        while (true)
        {
            if (Console.KeyAvailable)
            {
                switch (Console.ReadKey().Key)
                {
                    case ConsoleKey.Escape:
                        goto ESC;
                    case ConsoleKey.UpArrow:
                        car.SetVehicleSpeed(++targetSpeed);
                        Console.WriteLine($"---> target speed {targetSpeed:f1} km/h");
                        break;
                    case ConsoleKey.DownArrow:
                        car.SetVehicleSpeed(--targetSpeed);
                        Console.WriteLine($"---> target speed {targetSpeed:f1} km/h");
                        break;
                    case ConsoleKey.LeftArrow:
                        car.SetSteeringAngle(Angle.FromDegree(--targetAngle));
                        Console.WriteLine($"---> target angle {targetAngle:f1} deg");
                        break;
                    case ConsoleKey.RightArrow:
                        car.SetSteeringAngle(Angle.FromDegree(++targetAngle));
                        Console.WriteLine($"---> target angle {targetAngle:f1} deg");
                        break;
                }
            }
        }
    ESC:
        car.Dispose();
    }
}
