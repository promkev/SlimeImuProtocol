using System.Collections.Concurrent;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Numerics;
using System.Text;
using static SlimeImuProtocol.SlimeVR.FirmwareConstants;

namespace SlimeImuProtocol.SlimeVR
{
    public class UDPHandler : IDisposable
    {
        private static string _endpoint = "255.255.255.255";
        private static bool _handshakeOngoing = false;
        public static event EventHandler OnForceHandshake;
        public static event EventHandler OnForceDestroy;
        public static event EventHandler<string> OnServerDiscovered;
        private Stopwatch _timeSinceLastQuaternionDataPacket = new Stopwatch();
        private Stopwatch _timeSinceLastAccelerationDataPacket = new Stopwatch();
        private string _id;
        private byte[] _hardwareAddress;
        private int _supportedSensorCount;
        private PacketBuilder packetBuilder;
        private int slimevrPort = 6969;
        UdpClient udpClient;
        int handshakeCount = 1000;
        bool _active = true;
        private bool disposed;
        private EventHandler forceHandShakeDelegate;
        private Vector3 _lastAccelerationPacket;
        private Quaternion _lastQuaternion;

        private long _lastPacketReceivedTime = DateTimeOffset.UtcNow.ToUniversalTime().ToUnixTimeMilliseconds();
        private bool _isInitialized = false;
        private CancellationTokenSource _cts = new CancellationTokenSource();
        public bool IsDiscoveryOnly { get; set; } = false;

        // Packet telemetry counters surfaced to callers (UI / diagnostics). Stubs for now —
        // the current WatchdogLoop does not wire them; plumb through the send paths if
        // detailed metrics are needed later.
        public long PacketsSent => 0;
        public long SendFailures => 0;
        public bool ServerReachable => _isInitialized;

        public bool Active { get => _active; set => _active = value; }
        public static string Endpoint { get => _endpoint; set => _endpoint = value; }
        public static bool HandshakeOngoing { get => _handshakeOngoing; }

        /// <summary>
        /// Force the handshake state machine to re-run. Safe no-op if already initializing.
        /// </summary>
        public void Rehandshake() => TriggerHandshake();

        public UDPHandler(string firmware, byte[] hardwareAddress, BoardType boardType, ImuType imuType, McuType mcuType, MagnetometerStatus magnetometerStatus, int supportedSensorCount)
        {
            _id = Guid.NewGuid().ToString();
            _hardwareAddress = hardwareAddress;
            _supportedSensorCount = supportedSensorCount;
            packetBuilder = new PacketBuilder(firmware);
            ConfigureUdp();
            
            // Start the unified background loops
            Task.Run(() => ReceiveLoop(boardType, imuType, mcuType, magnetometerStatus, macAddress: hardwareAddress));
            Task.Run(() => WatchdogLoop(hardwareAddress, boardType, imuType, mcuType, magnetometerStatus, supportedSensorCount));
            Task.Run(() => HeartbeatLoop());

            forceHandShakeDelegate = delegate (object o, EventArgs e)
            {
                TriggerHandshake();
            };
            OnForceHandshake += forceHandShakeDelegate;
            OnForceDestroy += UDPHandler_OnForceDestroy;
        }

        private void TriggerHandshake()
        {
            _isInitialized = false;
        }

        private void UDPHandler_OnForceDestroy(object? sender, EventArgs e)
        {
            OnForceHandshake -= forceHandShakeDelegate;
            OnForceDestroy -= UDPHandler_OnForceDestroy;
            this?.Dispose();
        }

        public static void ForceUDPClientsToDoHandshake()
        {
            OnForceHandshake?.Invoke(new object(), EventArgs.Empty);
        }

        public static void ForceDestroy()
        {
            OnForceDestroy?.Invoke(new object(), EventArgs.Empty);
        }

        private async Task WatchdogLoop(byte[] hardwareAddress, BoardType boardType, ImuType imuType, McuType mcuType, MagnetometerStatus magnetometerStatus, int supportedSensorCount)
        {
            while (!disposed)
            {
                if (!_active)
                {
                    await Task.Delay(1000);
                    continue;
                }

                if (!_isInitialized)
                {
                    // Sequential Handshake Logic (Global Mutex)
                    while (_handshakeOngoing && !disposed)
                    {
                        await Task.Delay(1000);
                    }

                    if (disposed) break;

                    try
                    {
                        _handshakeOngoing = true;
                        Debug.WriteLine($"[UDPHandler] Starting Handshake for {_id}...");

                        while (!_isInitialized && _active && !disposed)
                        {
                            // Check if endpoint changed during handshake attempt
                            if (udpClient == null || _endpoint != Endpoint)
                            {
                                ConfigureUdp();
                            }

                            await udpClient.SendAsync(packetBuilder.BuildHandshakePacket(boardType, imuType, mcuType, magnetometerStatus, hardwareAddress));
                            await Task.Delay(1000);
                        }
                    }
                    catch (Exception ex)
                    {
                        Debug.WriteLine($"[UDPHandler] Handshake error for {_id}: {ex.Message}");
                    }
                    finally
                    {
                        _handshakeOngoing = false;
                    }

                    if (_isInitialized)
                    {
                        Debug.WriteLine($"[UDPHandler] Handshake Success for {_id}. Sending sensor info...");
                        for (int i = 0; i < _supportedSensorCount; i++)
                        {
                            await udpClient.SendAsync(packetBuilder.BuildSensorInfoPacket(imuType, TrackerPosition.NONE, TrackerDataType.ROTATION, (byte)i));
                        }

                        if (IsDiscoveryOnly)
                        {
                            Debug.WriteLine($"[UDPHandler] Discovery Only mode active. Disposing {_id}...");
                            Dispose();
                            break;
                        }
                    }
                }
                else if (udpClient != null && _endpoint != Endpoint)
                {
                    // Endpoint changed after initialization, re-handshake to new target
                    Debug.WriteLine($"[UDPHandler] Endpoint changed for {_id}. Re-configuring...");
                    _isInitialized = false;
                    ConfigureUdp();
                }

                // Connection persistence watchdog: 4 seconds
                long now = DateTimeOffset.UtcNow.ToUniversalTime().ToUnixTimeMilliseconds();
                if (_isInitialized && (now - _lastPacketReceivedTime > 4000))
                {
                    Debug.WriteLine($"[UDPHandler] Connection TIMEOUT for {_id}. Re-handshaking...");
                    _isInitialized = false;
                    _lastPacketReceivedTime = now; // Prevent instant re-trigger
                }

                await Task.Delay(1000);
            }
        }

        private async Task ReceiveLoop(BoardType boardType, ImuType imuType, McuType mcuType, MagnetometerStatus magnetometerStatus, byte[] macAddress)
        {
            while (!disposed)
            {
                try
                {
                    var result = await udpClient.ReceiveAsync();
                    _lastPacketReceivedTime = DateTimeOffset.UtcNow.ToUniversalTime().ToUnixTimeMilliseconds();

                    byte[] buffer = result.Buffer;
                    if (buffer.Length == 0) continue;

                    // Header check for handshake reply
                    string value = Encoding.UTF8.GetString(buffer);
                    if (value.Contains("Hey OVR =D 5"))
                    {
                        if (!_isInitialized)
                        {
                            _endpoint = result.RemoteEndPoint.Address.ToString();
                            udpClient.Connect(_endpoint, 6969);
                            _isInitialized = true;
                            Debug.WriteLine($"[UDPHandler] Got Discovery Response for {_id}: {_endpoint}");
                            OnServerDiscovered?.Invoke(null, _endpoint);
                        }
                        continue;
                    }

                    // Standard SlimeVR Packets (Type in first 4 bytes, Big Endian)
                    if (buffer.Length >= 4)
                    {
                        uint packetType = (uint)((buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3]);

                        if (packetType == 10) // PingPong
                        {
                            await udpClient.SendAsync(buffer, buffer.Length); // Echo back
                        }
                        else if (packetType == 1) // Server Heartbeat
                        {
                            await udpClient.SendAsync(packetBuilder.CreateHeartBeat()); // Reply with our heartbeat
                        }
                    }
                }
                catch (Exception ex) when (!disposed)
                {
                    Debug.WriteLine($"[UDPHandler] Receive Error: {ex.Message}");
                    await Task.Delay(100);
                }
            }
        }

        private async Task HeartbeatLoop()
        {
            while (!disposed)
            {
                if (_active && _isInitialized)
                {
                    try
                    {
                        await udpClient.SendAsync(packetBuilder.CreateHeartBeat());
                    }
                    catch { }
                }
                await Task.Delay(900);
            }
        }

        public void ConfigureUdp()
        {
            try
            {
                if (udpClient != null)
                {
                    udpClient?.Close();
                    udpClient?.Dispose();
                }
                _endpoint = Endpoint; // Cache the current static endpoint locally
                udpClient = new UdpClient();
                udpClient.Connect(_endpoint, 6969);
                Debug.WriteLine($"[UDPHandler] Configured UDP for {_id} -> {_endpoint}");
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[UDPHandler] ConfigureUdp error for {_id}: {ex.Message}");
            }
        }

        public async Task<bool> SetSensorRotation(Quaternion rotation, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await udpClient.SendAsync(packetBuilder.BuildRotationPacket(rotation, trackerId));
                _lastQuaternion = rotation;
            }
            return true;
        }

        public static bool QuatEqualsWithEpsilon(Quaternion a, Quaternion b)
        {
            const float epsilon = 0.0001f;
            return MathF.Abs(a.X - b.X) < epsilon
                && MathF.Abs(a.Y - b.Y) < epsilon
                && MathF.Abs(a.Z - b.Z) < epsilon
                && MathF.Abs(a.W - b.W) < epsilon;
        }

        public async Task<bool> SetSensorAcceleration(Vector3 acceleration, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await udpClient.SendAsync(packetBuilder.BuildAccelerationPacket(acceleration, trackerId));
                _timeSinceLastAccelerationDataPacket.Restart();
                _lastAccelerationPacket = acceleration;
            }
            return true;
        }

        public async Task<bool> SetThumbstick(Vector2 analogueThumbstick, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await udpClient.SendAsync(packetBuilder.BuildThumbstickPacket(analogueThumbstick, trackerId));
            }
            return true;
        }

        public async Task<bool> SetTrigger(float triggerAnalogue, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await udpClient.SendAsync(packetBuilder.BuildTriggerAnaloguePacket(triggerAnalogue, trackerId));
            }
            return true;
        }

        public async Task<bool> SetGrip(float gripAnalogue, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await udpClient.SendAsync(packetBuilder.BuildGripAnaloguePacket(gripAnalogue, trackerId));
            }
            return true;
        }

        public async Task<bool> SetSensorGyro(Vector3 gyro, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await udpClient.SendAsync(packetBuilder.BuildGyroPacket(gyro, trackerId));
            }
            return true;
        }
        public async Task<bool> SetSensorFlexData(float flexResistance, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await udpClient.SendAsync(packetBuilder.BuildFlexDataPacket(flexResistance, trackerId));
            }
            return true;
        }
        public async Task<bool> SendButton(UserActionType userActionType)
        {
            if (udpClient != null && _isInitialized)
            {
                await udpClient.SendAsync(packetBuilder.BuildButtonPushedPacket(userActionType));
            }
            return true;
        }
        public async Task<bool> SendControllerButton(ControllerButton userActionType, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await udpClient.SendAsync(packetBuilder.BuildControllerButtonPushedPacket(userActionType, trackerId));
            }
            return true;
        }

        public async Task<bool> SendPacket(byte[] packet)
        {
            if (udpClient != null && _isInitialized)
            {
                await udpClient.SendAsync(packet);
            }
            return true;
        }

        public async Task<bool> SetSensorBattery(float battery, float voltage)
        {
            if (udpClient != null && _isInitialized)
            {
                await udpClient.SendAsync(packetBuilder.BuildBatteryLevelPacket(battery, voltage));
            }
            return true;
        }

        public async Task<bool> SetSensorMagnetometer(Vector3 magnetometer, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await udpClient.SendAsync(packetBuilder.BuildMagnetometerPacket(magnetometer, trackerId));
            }
            return true;
        }

        public void Dispose()
        {
            try
            {
                if (!disposed)
                {
                    disposed = true;
                    _isInitialized = false;
                    _handshakeOngoing = false;
                    udpClient?.Close();
                    udpClient?.Dispose();
                    udpClient = null;
                }
            }
            catch { }
        }

        public void SendTrigger(float trigger, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                _ = udpClient.SendAsync(packetBuilder.BuildTriggerAnaloguePacket(trigger, trackerId));
            }
        }

        public void SendGrip(float grip, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                _ = udpClient.SendAsync(packetBuilder.BuildGripAnaloguePacket(grip, trackerId));
            }
        }
    }
}
