using System;
using System.Linq;
using System.Reactive.Linq;
using Reactive.Bindings;
using VePack.Utilities;
using AirSim;

namespace WpfApp
{

    public class MainWindowViewModel : IDisposable
    {

        // ------ fields ------ //

        private readonly AirSimAutoCar _car;


        // ------ properties ------ //

        public ReadOnlyReactivePropertySlim<TextDataTable?> Info { get; }
        public ReactivePropertySlim<string> TargetSpeed { set; get; } = new("0");
        public ReactiveCommand SpeedUpCommand { get; } = new();
        public ReactiveCommand SpeedDownCommand { get; } = new();
        public ReactiveCommand StartCommand { get; }
        public ReactiveCommand StopCommand { get; }


        // ------ constructors ------ //

        public MainWindowViewModel()
        {
            _car = new();
            StartCommand = _car.InfoUpdated.Select(x => x?.IsRunning is false).ToReactiveCommand().WithSubscribe(() => _car.Start());
            StopCommand = _car.InfoUpdated.Select(x => x?.IsRunning is true).ToReactiveCommand().WithSubscribe(() => _car.Stop());
            TargetSpeed.Subscribe(_ => _car?.SetVehicleSpeed(double.Parse(TargetSpeed.Value)));
            SpeedUpCommand.Subscribe(_ => TargetSpeed.Value = $"{(double.Parse(TargetSpeed.Value) + 0.5).OrBelow(10):f1}");
            SpeedDownCommand.Subscribe(_ => TargetSpeed.Value = $"{(double.Parse(TargetSpeed.Value) - 0.5).OrAbove(-10):f1}");
            Info = _car.InfoUpdated.Select(x => TextDataTable.FromCompositeInfo(x)).ToReadOnlyReactivePropertySlim();
        }


        // ------ public methods ------ //

        public void Dispose()
        {
            _car.Dispose();
            Info.Dispose();
            TargetSpeed.Dispose();
            StartCommand.Dispose();
            StopCommand.Dispose();
            SpeedUpCommand.Dispose();
            SpeedDownCommand.Dispose();
        }

    }
}
