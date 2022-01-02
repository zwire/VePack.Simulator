using System;
using AirSim;

namespace ConsoleApp
{
    internal class Program
    {
        static void Main(string[] args)
        {
            var prefix = "..\\..\\..\\..\\AirSim\\";
            var car = new AirSimAutoCar(prefix + "sim.map", prefix + "sim.pln");
            car.Start();
            car.InfoUpdated.Subscribe(x => Console.WriteLine($"Speed: {x.Vehicle.VehicleSpeed:f1}km/h, E: {x.Geo?.LateralError:f3}m, {x.Geo?.HeadingError.Degree:f0}deg"));
            var targetSpeed = 0.0;
            while (true)
            {
                if (Console.KeyAvailable)
                {
                    switch (Console.ReadKey().Key)
                    {
                        case ConsoleKey.Escape:
                            goto ESC;
                        case ConsoleKey.UpArrow:
                            targetSpeed++;
                            break;
                        case ConsoleKey.DownArrow:
                            targetSpeed--;
                            break;
                    }
                    car.SetVehicleSpeed(targetSpeed);
                    Console.WriteLine($"---> target speed {targetSpeed:f1}km/h");
                }
            }
        ESC:
            car.Stop();
            car.Dispose();
        }
    }
}
