using System;
using System.IO;
using AirSim;

namespace ConsoleApp
{
    internal class Program
    {
        static void Main(string[] args)
        {
            var prefix = "..\\..\\..\\..\\AirSim\\";
            var car = new AirSimAutoCar(prefix + "sim.map", prefix + "sim.pln", true);
            car.Start();

            var count = 0;
            while (File.Exists($"{count}.csv")) count++;
            //var sw = new StreamWriter($"{count}.csv");
            var targetSpeed = 0.0;
            var targetAngle = 0.0;
            car.InfoUpdated.Subscribe(x =>
            {
                var lateral = x.Geo.LateralError;
                var heading = x.Geo.HeadingError;
                var steer = x.Vehicle.SteeringAngle;
                var speed = x.Vehicle.VehicleSpeed;
                //sw?.WriteLine($"{lateral},{heading.Radian},{steer.Radian},{speed / 3.6}");
                Console.WriteLine($"Speed: {speed:f1}km/h, E: {lateral:f3}m, {heading.Degree:f0}deg");
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
            //sw.Close();
            //sw = null;
            car.Stop();
            car.Dispose();
        }
    }
}
