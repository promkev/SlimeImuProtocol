using SlimeImuProtocol.SlimeVR;
using System;
using System.Buffers.Binary;
using System.Numerics;
using static SlimeImuProtocol.SlimeVR.FirmwareConstants;

namespace SlimeImuProtocol.SlimeVR
{
    public class PacketBuilder
    {
        private string _identifierString = "Bootleg Tracker";
        private int _protocolVersion = 19;
        private long _packetId;

        // Per-packet sizes — allocated fresh on every Build* call. Previous version reused
        // instance-field buffers, which corrupted silently when two Tasks (e.g. the JSL
        // rotation callback and a concurrent accel send) built packets in parallel before
        // the first one had been sent. Fresh allocations trade ~90 B/packet garbage for
        // correctness.
        //
        // Zero-alloc hot path: HotBuf is a reusable buffer for synchronous send paths.
        // BuildInto* methods write directly into it; the caller sends via SendBufferDirect.
        // These must only be used from a single synchronous caller (the HID reader thread).
        internal readonly byte[] HotBuf = new byte[2048];

        private const int RotationBufferSize       = 4 + 8 + 1 + 1 + 16 + 1;
        private const int AccelerationBufferSize   = 4 + 8 + 12 + 1;
        internal const int RotationInnerSize       = 1 + 1 + 16 + 1;   // trackerId + dataType + quat + calibInfo
        internal const int AccelerationInnerSize   = 12 + 1;            // accel XYZ + trackerId
        private const int StickBufferSize          = 4 + 8 + 12 + 1;
        private const int TouchpadBufferSize       = 4 + 8 + 12 + 1;
        private const int GyroBufferSize           = 4 + 8 + 1 + 1 + 12 + 1;
        private const int MagnetometerBufferSize   = 4 + 8 + 1 + 1 + 12 + 1;
        private const int FlexDataBufferSize       = 4 + 8 + 1 + 4;
        private const int ButtonBufferSize         = 4 + 8 + 1;
        private const int BatteryBufferSize        = 4 + 8 + 4 + 4;
        private const int HapticBufferSize         = 4 + 8 + 4 + 4 + 1;
        private const int ControllerButtonSize     = 4 + 8 + 1 + 1;
        private const int TriggerAnalogueSize      = 4 + 8 + 4 + 1;
        private const int GripAnalogueSize         = 4 + 8 + 4 + 1;
        private const int HeartbeatBufferSize      = 4 + 8 + 1;

        public PacketBuilder(string fwString)
        {
            _identifierString = fwString;
        }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        internal long NextPacketId()
        {
            return System.Threading.Interlocked.Increment(ref _packetId) - 1;
        }

        public ReadOnlyMemory<byte> CreateHeartBeat()
        {
            var buf = new byte[HeartbeatBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.HEARTBEAT);
            w.WriteInt64(NextPacketId());
            w.WriteByte(0);
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildRotationPacket(Quaternion rotation, byte trackerId)
        {
            var buf = new byte[RotationBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.ROTATION_DATA);
            w.WriteInt64(NextPacketId());
            w.WriteByte(trackerId);
            w.WriteByte(1);
            w.WriteSingle(rotation.X);
            w.WriteSingle(rotation.Y);
            w.WriteSingle(rotation.Z);
            w.WriteSingle(rotation.W);
            w.WriteByte(0);
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildAccelerationPacket(Vector3 acceleration, byte trackerId)
        {
            var buf = new byte[AccelerationBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.ACCELERATION);
            w.WriteInt64(NextPacketId());
            w.WriteSingle(acceleration.X);
            w.WriteSingle(acceleration.Y);
            w.WriteSingle(acceleration.Z);
            w.WriteByte(trackerId);
            return buf.AsMemory(0, w.Position);
        }

        public int BuildRotationInto(Quaternion rotation, byte trackerId)
        {
            var w = new BigEndianWriter(HotBuf);
            w.WriteInt32((int)UDPPackets.ROTATION_DATA);
            w.WriteInt64(NextPacketId());
            w.WriteByte(trackerId);
            w.WriteByte(1);
            w.WriteSingle(rotation.X);
            w.WriteSingle(rotation.Y);
            w.WriteSingle(rotation.Z);
            w.WriteSingle(rotation.W);
            w.WriteByte(0);
            return w.Position;
        }

        public int BuildAccelerationInto(Vector3 acceleration, byte trackerId)
        {
            var w = new BigEndianWriter(HotBuf);
            w.WriteInt32((int)UDPPackets.ACCELERATION);
            w.WriteInt64(NextPacketId());
            w.WriteSingle(acceleration.X);
            w.WriteSingle(acceleration.Y);
            w.WriteSingle(acceleration.Z);
            w.WriteByte(trackerId);
            return w.Position;
        }

        public int BuildBundleInto(Quaternion rotation, Vector3 acceleration, byte trackerId)
        {
            // Layout:
            //  [0..3]   BUNDLE type (100)     — big-endian int32
            //  [4..11]  packet id              — big-endian int64
            //  [12..13] inner 1 length         — big-endian uint16  (4 + RotationInnerSize)
            //  [14..17] inner 1 type (17)      — big-endian int32
            //  [18..36] inner 1 payload        — RotationInnerSize bytes
            //  [37..38] inner 2 length         — big-endian uint16  (4 + AccelerationInnerSize)
            //  [39..42] inner 2 type (4)       — big-endian int32
            //  [43..55] inner 2 payload        — AccelerationInnerSize bytes

            int pos = 0;

            // BUNDLE header
            int pktType = (int)UDPPackets.BUNDLE;
            HotBuf[pos++] = (byte)((pktType >> 24) & 0xFF);
            HotBuf[pos++] = (byte)((pktType >> 16) & 0xFF);
            HotBuf[pos++] = (byte)((pktType >> 8) & 0xFF);
            HotBuf[pos++] = (byte)(pktType & 0xFF);

            long id = NextPacketId();
            HotBuf[pos++] = (byte)((id >> 56) & 0xFF);
            HotBuf[pos++] = (byte)((id >> 48) & 0xFF);
            HotBuf[pos++] = (byte)((id >> 40) & 0xFF);
            HotBuf[pos++] = (byte)((id >> 32) & 0xFF);
            HotBuf[pos++] = (byte)((id >> 24) & 0xFF);
            HotBuf[pos++] = (byte)((id >> 16) & 0xFF);
            HotBuf[pos++] = (byte)((id >> 8) & 0xFF);
            HotBuf[pos++] = (byte)(id & 0xFF);

            // Inner 1: ROTATION_DATA (type 17)
            short rLen = (short)(4 + RotationInnerSize);
            HotBuf[pos++] = (byte)((rLen >> 8) & 0xFF);
            HotBuf[pos++] = (byte)(rLen & 0xFF);
            HotBuf[pos++] = 0; HotBuf[pos++] = 0; HotBuf[pos++] = 0; HotBuf[pos++] = 17; // type 17

            // Rotation payload: trackerId + dataType + quaternion x,y,z,w + calibration
            HotBuf[pos++] = trackerId;
            HotBuf[pos++] = 1; // dataType
            WriteFloat(HotBuf, ref pos, rotation.X);
            WriteFloat(HotBuf, ref pos, rotation.Y);
            WriteFloat(HotBuf, ref pos, rotation.Z);
            WriteFloat(HotBuf, ref pos, rotation.W);
            HotBuf[pos++] = 0; // calibration

            // Inner 2: ACCELERATION (type 4)
            short aLen = (short)(4 + AccelerationInnerSize);
            HotBuf[pos++] = (byte)((aLen >> 8) & 0xFF);
            HotBuf[pos++] = (byte)(aLen & 0xFF);
            HotBuf[pos++] = 0; HotBuf[pos++] = 0; HotBuf[pos++] = 0; HotBuf[pos++] = 4; // type 4

            // Acceleration payload: x, y, z, trackerId
            WriteFloat(HotBuf, ref pos, acceleration.X);
            WriteFloat(HotBuf, ref pos, acceleration.Y);
            WriteFloat(HotBuf, ref pos, acceleration.Z);
            HotBuf[pos++] = trackerId;

            return pos;
        }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        private static void WriteFloat(byte[] buf, ref int pos, float value)
        {
            uint val = BitConverter.SingleToUInt32Bits(value);
            buf[pos++] = (byte)((val >> 24) & 0xFF);
            buf[pos++] = (byte)((val >> 16) & 0xFF);
            buf[pos++] = (byte)((val >> 8) & 0xFF);
            buf[pos++] = (byte)(val & 0xFF);
        }

        public ReadOnlyMemory<byte> BuildThumbstickPacket(Vector2 _analogueStick, byte trackerId)
        {
            var buf = new byte[StickBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.THUMBSTICK);
            w.WriteInt64(NextPacketId());
            w.WriteSingle(_analogueStick.X);
            w.WriteSingle(_analogueStick.Y);
            w.WriteSingle(0);
            w.WriteByte(trackerId);
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildTouchpadPacket(Vector2 _analogueTouchpad, byte trackerId)
        {
            var buf = new byte[TouchpadBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.THUMBSTICK);
            w.WriteInt64(NextPacketId());
            w.WriteSingle(_analogueTouchpad.X);
            w.WriteSingle(_analogueTouchpad.Y);
            w.WriteSingle(0);
            w.WriteByte(trackerId);
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildGyroPacket(Vector3 gyro, byte trackerId)
        {
            var buf = new byte[GyroBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.GYRO);
            w.WriteInt64(NextPacketId());
            w.WriteByte(trackerId);
            w.WriteByte(1);
            w.WriteSingle(gyro.X);
            w.WriteSingle(gyro.Y);
            w.WriteSingle(gyro.Z);
            w.WriteByte(0);
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildMagnetometerPacket(Vector3 m, byte trackerId)
        {
            var buf = new byte[MagnetometerBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.MAG);
            w.WriteInt64(NextPacketId());
            w.WriteByte(trackerId);
            w.WriteByte(1);
            w.WriteSingle(m.X);
            w.WriteSingle(m.Y);
            w.WriteSingle(m.Z);
            w.WriteByte(0);
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildFlexDataPacket(float flex, byte trackerId)
        {
            var buf = new byte[FlexDataBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.FLEX_DATA_PACKET);
            w.WriteInt64(NextPacketId());
            w.WriteByte(trackerId);
            w.WriteSingle(flex);
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildButtonPushedPacket(UserActionType action)
        {
            var buf = new byte[ButtonBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.CALIBRATION_ACTION);
            w.WriteInt64(NextPacketId());
            w.WriteByte((byte)action);
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildBatteryLevelPacket(float batteryPercent, float voltageVolts)
        {
            var buf = new byte[BatteryBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.BATTERY_LEVEL);
            w.WriteInt64(NextPacketId());
            w.WriteSingle(voltageVolts <= 0.1f ? 3.7f : voltageVolts);
            w.WriteSingle(Math.Clamp(batteryPercent / 100f, 0f, 1f));
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildHapticPacket(float intensity, int duration)
        {
            var buf = new byte[HapticBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.HAPTICS);
            w.WriteInt64(NextPacketId());
            w.WriteSingle(intensity);
            w.WriteInt32(duration);
            w.WriteByte(1);
            return buf.AsMemory(0, w.Position);
        }

        public byte[] BuildHandshakePacket(BoardType boardType, ImuType imuType, McuType mcuType, MagnetometerStatus magStatus, byte[] mac)
        {
            var idBytes = System.Text.Encoding.UTF8.GetBytes(_identifierString);
            int totalSize = 4 + 8 + 4 * 7 + 1 + idBytes.Length + mac.Length;
            var span = new byte[totalSize];

            var w = new BigEndianWriter(span);
            w.WriteInt32((int)UDPPackets.HANDSHAKE);
            w.WriteInt64(NextPacketId());
            w.WriteInt32((int)boardType);
            w.WriteInt32((int)imuType);
            w.WriteInt32((int)mcuType);
            w.WriteInt32((int)magStatus);
            w.WriteInt32(0);
            w.WriteInt32(0);
            w.WriteInt32(_protocolVersion);

            w.WriteByte((byte)idBytes.Length);
            idBytes.CopyTo(span.AsSpan(w.Position));
            w.Skip(idBytes.Length);

            mac.CopyTo(span.AsSpan(w.Position));
            w.Skip(mac.Length);

            return span;
        }

        public byte[] BuildSensorInfoPacket(ImuType imuType, TrackerPosition pos, TrackerDataType dataType, byte trackerId, MagnetometerStatus magStatus = MagnetometerStatus.NOT_SUPPORTED)
        {
            ushort sensorConfig = magStatus switch
            {
                MagnetometerStatus.ENABLED => 0x0003,
                MagnetometerStatus.DISABLED => 0x0002,
                _ => 0x0000,
            };
            var span = new byte[4 + 8 + 1 + 1 + 1 + 2 + 1 + 1 + 1];
            var w = new BigEndianWriter(span);
            w.SetPosition(0);
            w.WriteInt32((int)UDPPackets.SENSOR_INFO);
            w.WriteInt64(NextPacketId());
            w.WriteByte(trackerId);
            w.WriteByte(0);
            w.WriteByte((byte)imuType);
            w.WriteInt16((short)sensorConfig);
            w.WriteByte(0);
            w.WriteByte((byte)pos);
            w.WriteByte((byte)dataType);
            return span;
        }

        public ReadOnlyMemory<byte> BuildControllerButtonPushedPacket(ControllerButton action, byte trackerId)
        {
            var buf = new byte[ControllerButtonSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.CONTROLLER_BUTTON);
            w.WriteInt64(NextPacketId());
            w.WriteByte((byte)action);
            w.WriteByte(trackerId);
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildTriggerAnaloguePacket(float triggerAnalogue, byte trackerId)
        {
            var buf = new byte[TriggerAnalogueSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.TRIGGER);
            w.WriteInt64(NextPacketId());
            w.WriteSingle(triggerAnalogue);
            w.WriteByte(trackerId);
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildGripAnaloguePacket(float gripAnalogue, byte trackerId)
        {
            var buf = new byte[GripAnalogueSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.GRIP);
            w.WriteInt64(NextPacketId());
            w.WriteSingle(gripAnalogue);
            w.WriteByte(trackerId);
            return buf.AsMemory(0, w.Position);
        }

        public byte[] BuildFeatureFlagsPacket(params int[] enabledFlagBits)
        {
            int maxBit = 0;
            foreach (var b in enabledFlagBits) if (b > maxBit) maxBit = b;
            int flagByteCount = Math.Max(1, (maxBit / 8) + 1);

            var buf = new byte[4 + 8 + flagByteCount];
            var w = new BigEndianWriter(buf);
            w.SetPosition(0);
            w.WriteInt32((int)UDPPackets.FEATURE_FLAGS);
            w.WriteInt64(NextPacketId());
            foreach (var bit in enabledFlagBits)
            {
                int byteIdx = bit / 8;
                int bitIdx = bit % 8;
                buf[12 + byteIdx] |= (byte)(1 << bitIdx);
            }
            return buf;
        }

        public byte[] BuildPingPongPacket(int pingId)
        {
            var buf = new byte[4 + 8 + 4];
            var w = new BigEndianWriter(buf);
            w.SetPosition(0);
            w.WriteInt32((int)UDPPackets.PING_PONG);
            w.WriteInt64(NextPacketId());
            w.WriteInt32(pingId);
            return buf;
        }

        public byte[] BuildBundlePacket(params ReadOnlyMemory<byte>[] innerPackets)
        {
            int total = 4 + 8;
            foreach (var p in innerPackets) total += 2 + (p.Length - 8);

            var buf = new byte[total];
            var w = new BigEndianWriter(buf);
            w.SetPosition(0);
            w.WriteInt32((int)UDPPackets.BUNDLE);
            w.WriteInt64(NextPacketId());
            foreach (var p in innerPackets)
            {
                int innerLen = p.Length - 8;
                w.WriteInt16((short)innerLen);
                p.Span.Slice(0, 4).CopyTo(buf.AsSpan(w.Position));
                w.Skip(4);
                p.Span.Slice(12).CopyTo(buf.AsSpan(w.Position));
                w.Skip(p.Length - 12);
            }
            return buf;
        }
    }
}
