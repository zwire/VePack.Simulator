using System;
using AirSim;
using VePack.Utilities.Geometry;

namespace ConsoleApp
{
    internal class Program
    {
        static void Main(string[] args)
        {
            
            var car = new AirSimAutoCar();
            car.Start();
            car.InfoUpdated.Subscribe(x =>
            {
                var lateral = x.Geo.LateralError;
                var heading = x.Geo.HeadingError;
                var steer = x.Vehicle.SteeringAngle;
                var speed = x.Vehicle.VehicleSpeed;
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
                            car.SetSteeringAngle(new(--targetAngle, AngleType.Degree));
                            Console.WriteLine($"---> target angle {targetAngle:f1} deg");
                            break;
                        case ConsoleKey.RightArrow:
                            car.SetSteeringAngle(new(++targetAngle, AngleType.Degree));
                            Console.WriteLine($"---> target angle {targetAngle:f1} deg");
                            break;
                    }
                }
            }
        ESC:
            car.Dispose();
        }
    }
}
