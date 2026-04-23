namespace SlimeImuProtocol.SlimeProtocol {
    public class DeviceManager {
        private static readonly DeviceManager _instance = new DeviceManager();
        private readonly Dictionary<string, TrackerDevice> _devices = new Dictionary<string, TrackerDevice>();
        private int _nextLocalTrackerId;

        public static DeviceManager Instance => _instance;

        private DeviceManager() { }

        public void AddDevice(TrackerDevice newDevice) {
            lock (_devices) {
                _devices[newDevice.HardwareIdentifier] = newDevice;
            }
        }
        public int GetNextLocalTrackerId() {
            return Interlocked.Increment(ref _nextLocalTrackerId);
        }
    }
}
