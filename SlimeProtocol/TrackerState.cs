using SlimeImuProtocol.Utility;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SlimeImuProtocol.SlimeProtocol
{
    public static class TrackingEnvironment
    {
        // Always measured in world-space Y
        public static float FloorY { get; set; } = 0.0f;
        public static void UpdateFloor(params TrackerState[] trackers)
        {
            if (trackers == null || trackers.Length == 0) return;
            FloorY = trackers.Min(t => t.CalibratedPosition.Y);
        }
    }
    public class TrackerState
    {
        private Vector3 _position;
        private Vector3 _positionCalibration;

        private Quaternion _rotation;
        private Quaternion _rotationCalibration;
        private Vector3 _euler;

        private Vector3 _eulerCalibration;



        public int TrackerId { get; set; }
        public string BodyPart { get; set; }
        public string Ip { get; set; }

        public string Firmware { get; set; }

        public Quaternion Rotation
        {
            get
            {
                return _rotation;
            }
            set
            {
                _rotation = value;
                _euler = _rotation.QuaternionToEuler();
            }
        }

        public Quaternion RotationCalibrated
        {
            get
            {
                return Quaternion.Inverse(_rotationCalibration) * _rotation;
            }

        }

        public Vector3 CalibratedPosition { get { return _position - _positionCalibration; } }

        public Vector3 FloorRelativePosition
        {
            get { return new Vector3(CalibratedPosition.X, CalibratedPosition.Y - TrackingEnvironment.FloorY, CalibratedPosition.Z); }
        }

        public bool CloseToCalibratedY
        {
            get
            {
                return FloorRelativePosition.Y < 0.030f;
            }
        }
        public Vector3 SmoothRotation { get; set; }

        public Vector3 Euler { get => _euler; set => _euler = value; }
        public Vector3 CalibratedEuler { get => _eulerCalibration - _euler; }
        public Vector3 EulerCalibration { get => _eulerCalibration; set => _eulerCalibration = value; }
        public Vector3 Position { get => _position; set => _position = value; }
        public Vector3 PositionCalibration { get => _positionCalibration; set => _positionCalibration = value; }
        public Quaternion WorldRotation { get; internal set; }
        public Quaternion RotationCalibration { get => _rotationCalibration; set => _rotationCalibration = value; }
    }
}
