using System;
using Microsoft.Extensions.Configuration;

namespace GrpcClient
{
    public record AppOptions(string Address, string UserId, string Password)
    {

        public static AppOptions FromConfiguration(IConfiguration config)
        {

            var address = config["address"];
            if (address is null) throw new Exception("Address not specified.");

            var userId = config["user_id"] is { } uid ? uid : "";
            var password = config["password"] is { } pass ? pass : "";

            return new AppOptions(address, userId, password);

        }

    }
}
