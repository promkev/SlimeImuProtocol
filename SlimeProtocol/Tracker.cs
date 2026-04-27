using SlimeImuProtocol.SlimeVR;
using SlimeImuProtocol.Utility;
using System.Numerics;
using System.Text;
using static SlimeImuProtocol.SlimeVR.FirmwareConstants;

using System.Diagnostics;
using System.Threading;

namespace SlimeImuProtocol.SlimeProtocol {
    public class Tracker : IDisposable {
        public int TrackerNum { get; set; }
        public string Name { get; set; }
        public string DisplayName { get; set; }
        public bool HasRotation { get; set; }
        public bool HasAcceleration { get; set; }
        public bool UserEditable { get; set; }
        public ImuType ImuType { get; set; }
        public bool AllowFiltering { get; set; }
        public bool NeedsReset { get; set; }
        public bool NeedsMounting { get; set; }
        public bool UsesTimeout { get; set; }
        public MagnetometerStatus MagStatus { get; set; }

        public float BatteryLevel {
            get => _batteryLevel;
            set
            {
                if (MathF.Abs(value - _batteryLevel) < 0.01f) return;
                _batteryLevel = value;
                if (_ready)
                {
                    _ = _udpHandler.SetSensorBattery(_batteryLevel, BatteryVoltage);
                }
            }
        }
        public float BatteryVoltage { get; set; } = 3.7f;
        public float? Temperature { get; set; }
        public int SignalStrength { get; set; }

        public TrackerStatus Status = TrackerStatus.Disconnected;
        private UDPHandler _udpHandler;
        private bool _ready;
        private float _batteryLevel;
        private Quaternion _currentRotation;
        private Vector3 _currentAcceleration;
        private CancellationTokenSource _cts = new CancellationTokenSource();

        // --- Profiling ---
        private static long _totalSetBundleTicks;
        private static long _totalSetRotationTicks;
        private static int _bundleCount;
        private static int _rotationCount;

        public string FirmwareVersion { get; set; }
        public string HardwareIdentifier { get; set; }
        public BoardType BoardType { get; set; }
        public McuType McuType { get; set; }
        public Quaternion CurrentRotation { get => _currentRotation; set => _currentRotation = value; }
        public Vector3 CurrentAcceleration { get => _currentAcceleration; set => _currentAcceleration = value; }

        public Tracker(TrackerDevice device, int trackerNum, string name, string displayName, bool hasRotation, bool hasAcceleration,
            bool userEditable, ImuType imuType, bool allowFiltering, bool needsReset,
            bool needsMounting, bool usesTimeout, MagnetometerStatus magStatus) {
            TrackerNum = trackerNum;
            Name = name;
            DisplayName = displayName;
            HasRotation = hasRotation;
            HasAcceleration = hasAcceleration;
            UserEditable = userEditable;
            ImuType = imuType;
            AllowFiltering = allowFiltering;
            NeedsReset = needsReset;
            NeedsMounting = needsMounting;
            UsesTimeout = usesTimeout;
            MagStatus = magStatus;

            var token = _cts.Token;
            Task.Run(async () => {
                const int maxWaitMs = 60000;
                const int pollMs = 250;
                int elapsed = 0;
                while (device.FirmwareVersion == null) {
                    if (token.IsCancellationRequested) return;
                    try { await Task.Delay(pollMs, token); } catch (OperationCanceledException) { return; }
                    elapsed += pollMs;
                    if (elapsed >= maxWaitMs) return;
                }
                if (token.IsCancellationRequested) return;
                _udpHandler = new UDPHandler(device.FirmwareVersion + "_EsbToLan",
                 Encoding.UTF8.GetBytes(device.HardwareIdentifier), device.BoardType,
                 ImuType, device.McuType, MagStatus, 1);
                _ready = true;
            }, token);
        }

        public void TryInitialize() {
            if (FirmwareVersion != null && HardwareIdentifier != null) {
                _udpHandler = new UDPHandler(
                    FirmwareVersion + "_EsbToLan",
                    Encoding.UTF8.GetBytes(HardwareIdentifier),
                    BoardType,
                    ImuType,
                    McuType,
                    MagStatus,
                    1
                );
                _ready = true;
            }
        }

        public void SetRotation(Quaternion q) {
            if (_ready)
            {
                _currentRotation = q;
                var sw = Stopwatch.StartNew();
                _udpHandler?.SetSensorRotationSync(q, 0);
                sw.Stop();
                Interlocked.Add(ref _totalSetRotationTicks, sw.ElapsedTicks);
                int c = Interlocked.Increment(ref _rotationCount);
                if (c >= 200)
                {
                    double avgMs = (_totalSetRotationTicks / (double)c) / TimeSpan.TicksPerMillisecond;
                    Console.WriteLine($"[PERF] SetRotationSync avg {avgMs:F3} ms over {c}");
                    Interlocked.Exchange(ref _totalSetRotationTicks, 0);
                    Interlocked.Exchange(ref _rotationCount, 0);
                }
            }
        }

        public void SetAcceleration(Vector3 a) {
            if (_ready)
            {
                _currentAcceleration = a;
                _udpHandler?.SetSensorAccelerationSync(a, 0);
            }
        }

        public void SetBundle(Quaternion rotation, Vector3 acceleration) {
            if (_ready)
            {
                _currentRotation = rotation;
                _currentAcceleration = acceleration;
                var sw = Stopwatch.StartNew();
                _udpHandler?.SetSensorBundleSync(rotation, acceleration, 0);
                sw.Stop();
                Interlocked.Add(ref _totalSetBundleTicks, sw.ElapsedTicks);
                int c = Interlocked.Increment(ref _bundleCount);
                if (c >= 200)
                {
                    double avgMs = (_totalSetBundleTicks / (double)c) / TimeSpan.TicksPerMillisecond;
                    Console.WriteLine($"[PERF] SetBundleSync avg {avgMs:F3} ms over {c}");
                    Interlocked.Exchange(ref _totalSetBundleTicks, 0);
                    Interlocked.Exchange(ref _bundleCount, 0);
                }
            }
        }

        public void SetMagVector(Vector3 m) {
            if (_ready)
            {
                _udpHandler?.SetSensorMagnetometerSync(m, 0);
            }
        }

        public void Dispose() {
             _ready = false;
             try { _cts.Cancel(); } catch { }
             _udpHandler?.Dispose();
             _cts.Dispose();
        }
    }

    public enum TrackerStatus {
        OK,
        Disconnected
    }
}
