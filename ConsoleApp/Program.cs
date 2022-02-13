using System;
using AirSim;

namespace ConsoleApp
{
    internal class Program
    {
        static void Main(string[] args)
        {
            var prefix = "..\\..\\..\\..\\AirSim\\";
            var car = new AirSimAutoCar(prefix + "sim.map", prefix + "sim.pln", false);
            car.Start();
            car.InfoUpdated.Subscribe(x => Console.WriteLine($"Speed: {x.Vehicle.VehicleSpeed:f1}km/h, E: {x.Geo?.LateralError:f3}m, {x.Geo?.HeadingError.Degree:f0}deg"));
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
                            car.SetSteeringAngle(new(--targetAngle, VePack.Utilities.AngleType.Degree));
                            Console.WriteLine($"---> target speed {targetAngle:f1} deg");
                            break;
                        case ConsoleKey.RightArrow:
                            car.SetSteeringAngle(new(++targetAngle, VePack.Utilities.AngleType.Degree));
                            Console.WriteLine($"---> target speed {targetAngle:f1} deg");
                            break;
                    }
                }
            }
        ESC:
            car.Stop();
            car.Dispose();
        }
    }
}
