using System.Buffers.Binary;
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
        // 0 = idle, 1 = handshake in progress. Serialized via Interlocked.CompareExchange to
        // prevent two handlers from racing into simultaneous discovery.
        private static int _handshakeOngoingFlag = 0;
        public static event EventHandler OnForceHandshake;
        public static event EventHandler OnForceDestroy;
        public static event EventHandler<string> OnServerDiscovered;
        private Stopwatch _timeSinceLastQuaternionDataPacket = new Stopwatch();
        private Stopwatch _timeSinceLastAccelerationDataPacket = new Stopwatch();
        private string _id;
        private byte[] _hardwareAddress;
        private int _supportedSensorCount;
        internal PacketBuilder packetBuilder;
        private int slimevrPort = 6969;
        UdpClient udpClient;
        // Protects the udpClient reference during Configure/Dispose swaps. Reads take the lock
        // briefly to snapshot the reference, then operate on the snapshot — prevents
        // ObjectDisposedException races when Configure runs while a Send is in flight.
        private readonly object _udpClientLock = new object();
        int handshakeCount = 1000;
        bool _active = true;
        internal bool disposed;
        private EventHandler forceHandShakeDelegate;
        private Vector3 _lastAccelerationPacket;
        private Quaternion _lastQuaternion;

        private long _lastPacketReceivedTime = DateTimeOffset.UtcNow.ToUniversalTime().ToUnixTimeMilliseconds();
        private bool _isInitialized = false;
        private CancellationTokenSource _cts = new CancellationTokenSource();
        public bool IsDiscoveryOnly { get; set; } = false;

        private long _packetsSent;
        private long _sendFailures;
        // Set from the server's FEATURE_FLAGS reply. Gates use of BUNDLE (type 100). Default
        // false until the reply lands — otherwise first packets would silently drop on a
        // server that doesn't advertise PROTOCOL_BUNDLE_SUPPORT. Volatile so the send hot
        // path reads the latest value without a lock.
        private volatile bool _serverSupportsBundle;
        public long PacketsSent => System.Threading.Interlocked.Read(ref _packetsSent);
        public long SendFailures => System.Threading.Interlocked.Read(ref _sendFailures);
        public bool ServerReachable => _isInitialized;
        public bool ServerSupportsBundle => _serverSupportsBundle;

        private void SendInternalSync(ReadOnlyMemory<byte> payload)
        {
            if (disposed) return;
            UdpClient client;
            lock (_udpClientLock) { client = udpClient; }
            if (client == null) return;
            try
            {
                client.Client.Send(payload.Span);
                System.Threading.Interlocked.Increment(ref _packetsSent);
            }
            catch
            {
                System.Threading.Interlocked.Increment(ref _sendFailures);
            }
        }

        private void SendInternalSync(byte[] payload, int length)
        {
            if (disposed) return;
            UdpClient client;
            lock (_udpClientLock) { client = udpClient; }
            if (client == null) return;
            try
            {
                client.Client.Send(payload, length, SocketFlags.None);
                System.Threading.Interlocked.Increment(ref _packetsSent);
            }
            catch
            {
                System.Threading.Interlocked.Increment(ref _sendFailures);
            }
        }

        private async Task SendInternal(ReadOnlyMemory<byte> payload)
        {
            if (disposed) return;
            UdpClient client;
            lock (_udpClientLock) { client = udpClient; }
            if (client == null) return;
            try
            {
                await client.SendAsync(payload);
                System.Threading.Interlocked.Increment(ref _packetsSent);
            }
            catch
            {
                System.Threading.Interlocked.Increment(ref _sendFailures);
            }
        }

        private async Task SendInternal(byte[] payload)
        {
            if (disposed) return;
            UdpClient client;
            lock (_udpClientLock) { client = udpClient; }
            if (client == null) return;
            try
            {
                await client.SendAsync(payload, payload.Length);
                System.Threading.Interlocked.Increment(ref _packetsSent);
            }
            catch
            {
                System.Threading.Interlocked.Increment(ref _sendFailures);
            }
        }

        public bool Active { get => _active; set => _active = value; }
        public static string Endpoint { get => _endpoint; set => _endpoint = value; }
        public static bool HandshakeOngoing => System.Threading.Volatile.Read(ref _handshakeOngoingFlag) != 0;

        public void Rehandshake() => TriggerHandshake();

        public UDPHandler(string firmware, byte[] hardwareAddress, BoardType boardType, ImuType imuType, McuType mcuType, MagnetometerStatus magnetometerStatus, int supportedSensorCount)
        {
            _id = Guid.NewGuid().ToString();
            _hardwareAddress = hardwareAddress;
            _supportedSensorCount = supportedSensorCount;
            packetBuilder = new PacketBuilder(firmware);
            ConfigureUdp();

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
                    while (System.Threading.Interlocked.CompareExchange(ref _handshakeOngoingFlag, 1, 0) != 0 && !disposed)
                    {
                        await Task.Delay(1000);
                    }

                    if (disposed) break;

                    try
                    {
                        Debug.WriteLine($"[UDPHandler] Starting Handshake for {_id}...");

                        while (!_isInitialized && _active && !disposed)
                        {
                            if (udpClient == null || _endpoint != Endpoint)
                            {
                                ConfigureUdp();
                            }

                            await SendInternal(packetBuilder.BuildHandshakePacket(boardType, imuType, mcuType, magnetometerStatus, hardwareAddress));
                            await Task.Delay(1000);
                        }
                    }
                    catch (Exception ex)
                    {
                        Debug.WriteLine($"[UDPHandler] Handshake error for {_id}: {ex.Message}");
                    }
                    finally
                    {
                        System.Threading.Interlocked.Exchange(ref _handshakeOngoingFlag, 0);
                    }

                    if (_isInitialized)
                    {
                        Debug.WriteLine($"[UDPHandler] Handshake Success for {_id}. Sending sensor info...");
                        for (int i = 0; i < _supportedSensorCount; i++)
                        {
                            await SendInternal(packetBuilder.BuildSensorInfoPacket(imuType, TrackerPosition.NONE, TrackerDataType.ROTATION, (byte)i, magnetometerStatus));
                        }

                        try
                        {
                            await SendInternal(packetBuilder.BuildFeatureFlagsPacket(UDPPackets.FeatureFlagBits.PROTOCOL_BUNDLE_SUPPORT));
                        }
                        catch (Exception ex) { Debug.WriteLine($"[UDPHandler] FeatureFlags send failed: {ex.Message}"); }

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
                    Debug.WriteLine($"[UDPHandler] Endpoint changed for {_id}. Re-configuring...");
                    _isInitialized = false;
                    ConfigureUdp();
                }

                long now = DateTimeOffset.UtcNow.ToUniversalTime().ToUnixTimeMilliseconds();
                if (_isInitialized && (now - _lastPacketReceivedTime > 4000))
                {
                    Debug.WriteLine($"[UDPHandler] Connection TIMEOUT for {_id}. Re-handshaking...");
                    _isInitialized = false;
                    _lastPacketReceivedTime = now;
                }

                await Task.Delay(1000);
            }
        }

        private async Task ReceiveLoop(BoardType boardType, ImuType imuType, McuType mcuType, MagnetometerStatus magnetometerStatus, byte[] macAddress)
        {
            while (!disposed)
            {
                UdpClient client;
                lock (_udpClientLock) { client = udpClient; }
                if (client == null)
                {
                    await Task.Delay(100);
                    continue;
                }
                try
                {
                    var result = await client.ReceiveAsync();
                    _lastPacketReceivedTime = DateTimeOffset.UtcNow.ToUniversalTime().ToUnixTimeMilliseconds();

                    byte[] buffer = result.Buffer;
                    if (buffer.Length == 0) continue;

                    uint packetType = buffer.Length >= 4
                        ? (uint)((buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3])
                        : uint.MaxValue;

                    bool looksLikeHandshakeAscii =
                        packetType == UDPPacketsIn.RECEIVE_HANDSHAKE
                        || (buffer.Length >= 12 && Encoding.ASCII.GetString(buffer, 0, Math.Min(buffer.Length, 16)).Contains("Hey OVR"));

                    if (looksLikeHandshakeAscii && !_isInitialized)
                    {
                        _endpoint = result.RemoteEndPoint.Address.ToString();
                        lock (_udpClientLock) { udpClient?.Connect(_endpoint, 6969); }
                        _isInitialized = true;
                        Debug.WriteLine($"[UDPHandler] Got Discovery Response for {_id}: {_endpoint}");
                        OnServerDiscovered?.Invoke(null, _endpoint);
                        continue;
                    }

                    if (packetType == UDPPackets.PING_PONG)
                    {
                        await SendInternal(buffer);
                    }
                    else if (packetType == UDPPacketsIn.RECEIVE_HEARTBEAT)
                    {
                        await SendInternal(packetBuilder.CreateHeartBeat());
                    }
                    else if (packetType == UDPPackets.FEATURE_FLAGS)
                    {
                        if (buffer.Length >= 13)
                        {
                            byte flagByte0 = buffer[12];
                            bool bundleBit = (flagByte0 & (1 << UDPPackets.FeatureFlagBits.PROTOCOL_BUNDLE_SUPPORT)) != 0;
                            _serverSupportsBundle = bundleBit;
                            Debug.WriteLine($"[UDPHandler] Server FEATURE_FLAGS: bundle={bundleBit}");
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
                    await SendInternal(packetBuilder.CreateHeartBeat());
                }
                await Task.Delay(900);
            }
        }

        public void ConfigureUdp()
        {
            UdpClient oldClient;
            UdpClient newClient = null;
            lock (_udpClientLock)
            {
                oldClient = udpClient;
                udpClient = null;
            }
            try
            {
                _endpoint = Endpoint;
                newClient = new UdpClient();
                newClient.Connect(_endpoint, 6969);
                lock (_udpClientLock) { udpClient = newClient; }
                Debug.WriteLine($"[UDPHandler] Configured UDP for {_id} -> {_endpoint}");
            }
            catch (Exception ex)
            {
                try { newClient?.Dispose(); } catch { }
                Debug.WriteLine($"[UDPHandler] ConfigureUdp error for {_id}: {ex.Message}");
            }
            finally
            {
                try { oldClient?.Close(); } catch { }
                try { oldClient?.Dispose(); } catch { }
            }
        }

        public void SetSensorRotationSync(Quaternion rotation, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                SendInternalSync(packetBuilder.BuildRotationPacket(rotation, trackerId));
                _lastQuaternion = rotation;
            }
        }

        public void SetSensorAccelerationSync(Vector3 acceleration, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                SendInternalSync(packetBuilder.BuildAccelerationPacket(acceleration, trackerId));
                _timeSinceLastAccelerationDataPacket.Restart();
                _lastAccelerationPacket = acceleration;
            }
        }

        public void SetSensorMagnetometerSync(Vector3 magnetometer, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                SendInternalSync(packetBuilder.BuildMagnetometerPacket(magnetometer, trackerId));
            }
        }

        public void SetSensorBundleSync(Quaternion rotation, Vector3 acceleration, byte trackerId)
        {
            if (udpClient == null || !_isInitialized) return;
            var rot = packetBuilder.BuildRotationPacket(rotation, trackerId);
            var acc = packetBuilder.BuildAccelerationPacket(acceleration, trackerId);
            if (_serverSupportsBundle)
            {
                SendInternalSync(packetBuilder.BuildBundlePacket(rot, acc));
            }
            else
            {
                SendInternalSync(acc);
                SendInternalSync(rot);
            }
            _lastQuaternion = rotation;
            _lastAccelerationPacket = acceleration;
        }

        /// <summary>
        /// Zero-allocation send path for the hot HID reader thread. Builds
        /// ROTATION_DATA + ACCELERATION (bundled or separate) directly into
        /// packetBuilder.HotBuf, then sends from the buffer. No per-call
        /// byte[] allocations.
        /// </summary>
        internal void SetSensorBundleZero(Quaternion rotation, Vector3 acceleration, byte trackerId)
        {
            if (udpClient == null || !_isInitialized || disposed) return;

            if (_serverSupportsBundle)
            {
                int len = packetBuilder.BuildBundleInto(rotation, acceleration, trackerId);
                SendInternalSync(packetBuilder.HotBuf, len);
            }
            else
            {
                int len = packetBuilder.BuildAccelerationInto(acceleration, trackerId);
                SendInternalSync(packetBuilder.HotBuf, len);
                len = packetBuilder.BuildRotationInto(rotation, trackerId);
                SendInternalSync(packetBuilder.HotBuf, len);
            }

            _lastQuaternion = rotation;
            _lastAccelerationPacket = acceleration;
            _timeSinceLastAccelerationDataPacket.Restart();
        }

        internal void SendHotBuf(int length)
        {
            if (udpClient != null && !disposed)
                SendInternalSync(packetBuilder.HotBuf, length);
        }

        public async Task<bool> SetThumbstick(Vector2 analogueThumbstick, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
                await SendInternal(packetBuilder.BuildThumbstickPacket(analogueThumbstick, trackerId));
            return true;
        }

        public async Task<bool> SetTrigger(float triggerAnalogue, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
                await SendInternal(packetBuilder.BuildTriggerAnaloguePacket(triggerAnalogue, trackerId));
            return true;
        }

        public async Task<bool> SetGrip(float gripAnalogue, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
                await SendInternal(packetBuilder.BuildGripAnaloguePacket(gripAnalogue, trackerId));
            return true;
        }

        public async Task<bool> SetSensorGyro(Vector3 gyro, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
                await SendInternal(packetBuilder.BuildGyroPacket(gyro, trackerId));
            return true;
        }

        public async Task<bool> SetSensorFlexData(float flexResistance, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
                await SendInternal(packetBuilder.BuildFlexDataPacket(flexResistance, trackerId));
            return true;
        }

        public async Task<bool> SendButton(UserActionType userActionType)
        {
            if (udpClient != null && _isInitialized)
                await SendInternal(packetBuilder.BuildButtonPushedPacket(userActionType));
            return true;
        }

        public async Task<bool> SendControllerButton(ControllerButton userActionType, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
                await SendInternal(packetBuilder.BuildControllerButtonPushedPacket(userActionType, trackerId));
            return true;
        }

        public async Task<bool> SendPacket(byte[] packet)
        {
            if (udpClient != null && _isInitialized)
                await SendInternal(packet);
            return true;
        }

        public async Task<bool> SetSensorBattery(float battery, float voltage)
        {
            if (udpClient != null && _isInitialized)
                await SendInternal(packetBuilder.BuildBatteryLevelPacket(battery, voltage));
            return true;
        }

        public async Task<bool> SetSensorMagnetometer(Vector3 magnetometer, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
                await SendInternal(packetBuilder.BuildMagnetometerPacket(magnetometer, trackerId));
            return true;
        }

        public async Task<bool> SendBundle(params ReadOnlyMemory<byte>[] innerPackets)
        {
            if (udpClient != null && _isInitialized && innerPackets != null && innerPackets.Length > 0)
                await SendInternal(packetBuilder.BuildBundlePacket(innerPackets));
            return true;
        }

        public async Task<bool> SetSensorBundle(Quaternion rotation, Vector3 acceleration, byte trackerId)
        {
            if (udpClient == null || !_isInitialized) return true;
            var rot = packetBuilder.BuildRotationPacket(rotation, trackerId);
            var acc = packetBuilder.BuildAccelerationPacket(acceleration, trackerId);
            if (_serverSupportsBundle)
            {
                await SendInternal(packetBuilder.BuildBundlePacket(rot, acc));
            }
            else
            {
                await SendInternal(acc);
                await SendInternal(rot);
            }
            _lastQuaternion = rotation;
            _lastAccelerationPacket = acceleration;
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
                    System.Threading.Interlocked.Exchange(ref _handshakeOngoingFlag, 0);
                    UdpClient c;
                    lock (_udpClientLock) { c = udpClient; udpClient = null; }
                    try { c?.Close(); } catch { }
                    try { c?.Dispose(); } catch { }
                }
            }
            catch { }
        }

        public void SendTrigger(float trigger, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
                _ = SendInternal(packetBuilder.BuildTriggerAnaloguePacket(trigger, trackerId));
        }

        public void SendGrip(float grip, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
                _ = SendInternal(packetBuilder.BuildGripAnaloguePacket(grip, trackerId));
        }
    }
}
