using System;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using ZLogger;

namespace GrpcClient
{
    internal class Program
    {
        static void Main(string[] args)
        {
            CreateHostBuilder(args).Build().Run();
        }

        static IHostBuilder CreateHostBuilder(string[] args)
        {
            var appDir = AppDomain.CurrentDomain.BaseDirectory;

            return Host.CreateDefaultBuilder(args)
                .UseConsoleLifetime(options => options.SuppressStatusMessages = true)
                .ConfigureHostConfiguration(hostConfig => hostConfig.SetBasePath(appDir))
                .ConfigureLogging(logging =>
                {
                    logging.ClearProviders();
                    logging.AddZLoggerConsole();
                    //var logDir = Path.Combine(appDir, "log");
                    //if (!Directory.Exists(logDir)) Directory.CreateDirectory(logDir);
                    //logging.AddZLoggerFile($"{logDir}/{DateTimeOffset.Now:yyyy-MM-dd_HH-mm-ss}.log");
                })
                .ConfigureServices(services =>
                {
                    services.AddHostedService<Worker>();
                });
        }
    }
}
