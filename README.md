# SlimeImuProtocol

C# library that speaks the [SlimeVR](https://slimevr.dev) tracker UDP protocol. Used as a submodule by Everything_To_IMU_SlimeVR and similar bridge apps that want to register a virtual tracker with a SlimeVR Server over the local network.

Original project by [@Sebane1](https://github.com/Sebane1/SlimeImuProtocol).

## What it does

- Runs the SlimeVR tracker-side handshake against a server on UDP 6969.
- Builds + sends the standard packet set: rotation, acceleration, gyro, magnetometer, battery, handshake, sensor info, heartbeat, haptics, controller buttons/thumbstick/trigger/grip.
- Handles incoming server packets: ping/pong echo, feature flags capability exchange.
- Negotiates protocol features and can send multi-packet bundles (type 100) when the server advertises support.
- Auto-reconnect with exponential backoff; crash-safe against ICMP port-unreachable storms on Windows.

## Wire format

- All multi-byte values big-endian.
- Every outbound packet: `int32 type + int64 packetId + payload`.
- Protocol version 19 (matches current SlimeVR firmware).

Packet types live in `SlimeProtocol/FirmwareConstants.cs` under `UDPPackets`.

## Usage

```csharp
using SlimeImuProtocol.SlimeVR;
using static SlimeImuProtocol.SlimeVR.FirmwareConstants;

var handler = new UDPHandler(
    firmware:        "0.7.2-MyBridge",
    hardwareAddress: new byte[] { 0x02, 0xA1, 0xB2, 0xC3, 0xD4, 0xE5 },
    boardType:       BoardType.CUSTOM,
    imuType:         ImuType.BMI270,
    mcuType:         McuType.UNKNOWN,
    magnetometerStatus: MagnetometerStatus.NOT_SUPPORTED,
    supportedSensorCount: 1);

handler.Active = true;

// On each IMU sample:
await handler.SetSensorRotation(quaternion, trackerId: 0);
await handler.SetSensorAcceleration(accelVector, trackerId: 0);
```

The handshake, heartbeat, and inbound listener start automatically from the constructor.

## Requirements

- .NET 10.
- A running SlimeVR Server reachable by UDP broadcast (or on `UDPHandler.Endpoint`).

## Build

```bash
dotnet build SlimeImuProtocol.csproj -c Debug
```

## License

Follows the upstream project's license.
