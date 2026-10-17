"""Reproduce Crush's real sensor-driver output from Stonefish measurements.

Each conversion mirrors a driver in robosub-ros2 main@55b6d17, quirks included, so the
production EKF, controls and task planning see the same messages as on the vehicle.
"""

import io
import math

import numpy as np
from PIL import Image

# ROS base_link is forward/left/up; Stonefish bodies are forward/right/down. This is also a pi roll.
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])
ATMOSPHERE = 101325.0
# offboard_comms/sketches/peripheral: MS5837 library atmosphere and Pressure.hpp fluid density.
SENSOR_ATMOSPHERE = 101300.0
SENSOR_DENSITY = 997.0
GRAVITY = 9.80665
DEPTH_LIMIT = 7.0
COVARIANCE = 0.01


def rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def euler(matrix):
    return [
        math.atan2(matrix[2, 1], matrix[2, 2]),
        math.asin(max(-1.0, min(1.0, -matrix[2, 0]))),
        math.atan2(matrix[1, 0], matrix[0, 0]),
    ]


def sim_position(config, position):
    """Stonefish link-frame position of a point given relative to robosub-ros2 corner_link."""
    base = np.asarray(config['corner_link'], dtype=float) + np.asarray(position, dtype=float)
    return (np.asarray(config['base_link'], dtype=float) + FLU_TO_FRD @ base).tolist()


def sim_rpy(rpy, reports_frame_axes=False):
    """Stonefish orientation, in the FRD body, of a frame given in robosub-ros2 base_link axes.

    Scalar sensors report in forward/right/down axes of their mount. A Stonefish DVL instead
    points its beams along -z, so its frame is the robosub-ros2 frame itself.
    """
    matrix = FLU_TO_FRD @ rotation(rpy) @ FLU_TO_FRD
    return euler(matrix @ FLU_TO_FRD if reports_frame_axes else matrix)


# A Stonefish camera looks along its +z with +y down, like a ROS optical frame; this rotates
# robosub-ros2's forward/left/up camera frame onto that optical frame.
OPTICAL = rotation((-math.pi / 2, 0.0, -math.pi / 2))


def sim_camera_rpy(rpy):
    """Stonefish orientation, in the FRD body, of a camera whose robosub-ros2 frame looks along +x."""
    return euler(FLU_TO_FRD @ rotation(rpy) @ OPTICAL)


def quaternion(rpy):
    """ROS fixed-axis roll/pitch/yaw as an (x, y, z, w) quaternion."""
    cr, sr = math.cos(rpy[0] / 2), math.sin(rpy[0] / 2)
    cp, sp = math.cos(rpy[1] / 2), math.sin(rpy[1] / 2)
    cy, sy = math.cos(rpy[2] / 2), math.sin(rpy[2] / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


# vectornav/src/vn_sensor_msgs.cc with use_enu false: q_ned2enu.setRPY(M_PI, 0.0, -M_PI / 2).
VN_NED_TO_ENU = quaternion((math.pi, 0.0, -math.pi / 2))


def vectornav_orientation(body, mount):
    """Published /vectornav/imu orientation: q_ned2enu * (NED attitude of the VN100 body).

    `body` is the Stonefish link attitude in NED and `mount` the VN100 mount in that link.
    """
    return multiply(VN_NED_TO_ENU, multiply(body, mount))


def jpeg(width, height, rgb, quality=90):
    """JPEG bytes of a Stonefish rgb8 image, as the USB camera driver sends them."""
    output = io.BytesIO()
    Image.frombuffer('RGB', (width, height), bytes(rgb), 'raw', 'RGB', 0, 1).save(output, 'JPEG', quality=quality)
    return output.getvalue()


class Depth:
    """peripheral_sensors.py: median of three Arduino depths; out-of-range readings repeat the last value."""

    def __init__(self):
        self.values = []

    def update(self, gauge_pressure):
        """Return the published position.z (negative underwater), or None before a valid reading."""
        # MS5837::depth() on absolute pressure; the sketch prints it with two decimals.
        reading = round((gauge_pressure + ATMOSPHERE - SENSOR_ATMOSPHERE) / (SENSOR_DENSITY * GRAVITY), 2)
        if -DEPTH_LIMIT <= reading <= DEPTH_LIMIT:
            self.values = self.values[1:] + [reading] if self.values else [reading] * 3
        return -sorted(self.values)[1] if self.values else None


class Gyro:
    """gyro.py: counter-clockwise yaw rate and its integral from zero, wrapped to [-180, 180) degrees."""

    def __init__(self, rate):
        self.period = 1.0 / rate
        self.integral = 0.0

    def update(self, frd_yaw_rate):
        """Return (angular.z in rad/s, yaw orientation as x, y, z, w)."""
        rate = -frd_yaw_rate
        self.integral += math.degrees(rate) * self.period
        yaw = math.radians((self.integral + 180.0) % 360.0 - 180.0)
        return rate, (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
