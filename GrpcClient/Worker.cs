using System;
using System.Net.Http;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using System.Reactive.Linq;
using System.Reactive.Threading.Tasks;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;
using Grpc.Core;
using Grpc.Net.Client;
using ZLogger;
using AirSim;

namespace GrpcClient
{
    public class Worker : BackgroundService
    {

        private readonly IHostApplicationLifetime _hostAppLifetime;
        private readonly IConfiguration _config;
        private readonly ILogger<Worker> _logger;

        private readonly AirSimAutoCar _car;
        private float _targetVehicleSpeed;


        public Worker(IHostApplicationLifetime hostAppLifetime, IConfiguration config, ILogger<Worker> logger)
        {
            _hostAppLifetime = hostAppLifetime;
            _config = config;
            _logger = logger;
            var prefix = "..\\..\\..\\..\\AirSim\\";
            _car = new AirSimAutoCar(prefix + "sim.map", prefix + "sim.pln");
            //_car.InfoUpdated.Subscribe(x => _logger.ZLogInformation($"log :: Speed: {x.Vehicle.VehicleSpeed:f1}km/h, E: {x.Geo?.LateralError:f3}m, {x.Geo?.HeadingError.Degree:f0}deg"));
        }

        protected override async Task ExecuteAsync(CancellationToken stoppingToken)
        {

            AppOptions appOptions;
            try
            {
                appOptions = AppOptions.FromConfiguration(_config);
            }
            catch (Exception e)
            {
                _logger.ZLogInformation("Application Configuration Error:");
                _logger.ZLogInformation(e.Message);
                _hostAppLifetime.StopApplication();
                return;
            }

            var ctr = stoppingToken.Register(() =>
            {
                _logger.ZLogInformation("");
                _logger.ZLogInformation("Canceled.");
            });

            _logger.ZLogInformation($"User ID: {appOptions.UserId}");
            _logger.ZLogInformation($"Target Address: {appOptions.Address}");
            _logger.ZLogInformation("");

            // allow untrusted/invalid connection
            var httpHandler = new HttpClientHandler();
            httpHandler.ServerCertificateCustomValidationCallback =
                HttpClientHandler.DangerousAcceptAnyServerCertificateValidator;

            using var channel = GrpcChannel.ForAddress(appOptions.Address, new() { HttpHandler = httpHandler });

            try
            {
                var cts = CancellationTokenSource.CreateLinkedTokenSource(stoppingToken);
                var req = new AcceptLoginRequest { UserId = appOptions.UserId, Password = appOptions.Password };
                var client = new RemoteMonitor.RemoteMonitorClient(channel);
                var id = client.AcceptLogin(req).SessionId;
                if (id is "") throw new Exception("Failed to connect!");

                var propertyCall = client.AcceptTractorProperty(null, null, cts.Token);
                var commandCall = client.NotifyOperationCommand(new() { SessionId = id }, new CallOptions(null, null, cts.Token));
                var optionCall = client.NotifyOperationOption(new() { SessionId = id }, new CallOptions(null, null, cts.Token));
                var releaseCall = client.NotifyReleaseAsync(new() { SessionId = id }, new CallOptions(null, null, cts.Token));

                var tasks = new List<Task>
                {
                    SendTractorInformation(id, propertyCall, cts),
                    ReceiveOperationCommand(commandCall, cts),
                    ReceiveOperationOption(optionCall, cts),
                    ReserveReleaseNotification(releaseCall, cts)
                };
                await Task.WhenAll(tasks).ConfigureAwait(false);

            }
            catch (RpcException e)
            {
                _logger.ZLogInformation("!!!!  gRPC Error: status {0}", e.StatusCode);
            }

            await ctr.DisposeAsync().ConfigureAwait(false);
            _hostAppLifetime.StopApplication();
            _logger.ZLogInformation("");
            _logger.ZLogInformation("Completed.");

        }

        private async Task SendTractorInformation(
            string sessionId,
            AsyncClientStreamingCall<AcceptTractorPropertyRequest, AcceptTractorPropertyReply> propertyCall,
            CancellationTokenSource cts
        )
        {
            await _car.InfoUpdated.Sample(TimeSpan.FromMilliseconds(500))
                .TakeUntil(_ => cts.Token.IsCancellationRequested)
                .Where(x => x is not null)
                .Select(async x =>
                {
                    var property = new AcceptTractorPropertyRequest()
                    {
                        SessionId = sessionId,
                        IsRunning = x.IsRunning,
                        BehaviorMode = x.IsRunning ? GrpcBehaviorMode.Work : GrpcBehaviorMode.Unknown,
                        TargetVehicleSpeed = _targetVehicleSpeed,
                        ActualVehicleSpeed = (float)x.Vehicle.VehicleSpeed,
                        Latitude = x.Gnss.Latitude,
                        Longitude = x.Gnss.Longitude
                    };
                    await propertyCall.RequestStream.WriteAsync(property);
                    _logger.ZLogInformation($"client --> server : {property}");
                })
                .ToTask();
        }

        private async Task ReceiveOperationCommand(
            AsyncServerStreamingCall<NotifyOperationCommandReply> commandCall,
            CancellationTokenSource cts
        )
        {
            await foreach (var command in commandCall.ResponseStream.ReadAllAsync().ConfigureAwait(false))
            {
                _logger.ZLogInformation($"client <-- server : <Command> {command.OperationCommand}");
                if (cts.IsCancellationRequested) break;
                switch (command.OperationCommand)
                {
                    case GrpcOperationCommand.Go:
                        _car.Start();
                        break;
                    case GrpcOperationCommand.Stop:
                    case GrpcOperationCommand.Free:
                        _car.Stop();
                        break;
                }
            }
        }

        private async Task ReceiveOperationOption(
            AsyncServerStreamingCall<NotifyOperationOptionReply> optionCall,
            CancellationTokenSource cts
        )
        {
            await foreach (var option in optionCall.ResponseStream.ReadAllAsync().ConfigureAwait(false))
            {
                if (cts.IsCancellationRequested) break;
                switch (option.ChosenCase)
                {
                    case NotifyOperationOptionReply.ChosenOneofCase.VehicleSpeed:
                        _targetVehicleSpeed = option.VehicleSpeed;
                        _car.SetVehicleSpeed(option.VehicleSpeed);
                        _logger.ZLogInformation($"client <-- server : <Vehicle Speed> {option.VehicleSpeed}");
                        break;
                    default:
                        break;
                }
            }
        }

        private async Task ReserveReleaseNotification(
            AsyncUnaryCall<NotifyReleaseReply> releaseCall,
            CancellationTokenSource cts
        )
        {
            await releaseCall.ResponseAsync;
            _logger.ZLogInformation($"client <-- server : <Release>");
            _car.Stop();
            _car.Dispose();
            cts.Cancel();
        }
    }

}