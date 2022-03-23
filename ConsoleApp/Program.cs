using System;
using System.IO;
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

            var count = 0;
            while (File.Exists($"{count}.csv")) count++;
            StreamWriter sw = null;
            //sw = new StreamWriter($"{count}.csv");
            var targetSpeed = 0.0;
            var targetAngle = 0.0;
            car.InfoUpdated.Subscribe(x =>
            {
                var lateral = x.Geo.LateralError;
                var heading = x.Geo.HeadingError;
                var steer = x.Vehicle.SteeringAngle;
                var speed = x.Vehicle.VehicleSpeed;
                sw?.WriteLine($"{lateral},{heading.Radian},{steer.Radian},{speed / 3.6}");
                Console.WriteLine($"Speed: {speed:f1}km/h, E: {lateral:f3}m, {heading.Degree:f1}deg");
            });
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
            sw?.Close();
            sw = null;
            car.Dispose();
        }
    }
}
