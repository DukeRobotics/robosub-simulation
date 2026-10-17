import math
from pathlib import Path

import numpy as np
import pytest
from robosub_simulation.robot import build_robot, load_robot_config
from robosub_simulation.sensors import (
    Depth,
    Gyro,
    quaternion,
    rotation,
    sim_position,
    sim_rpy,
    vectornav_orientation,
)

CONFIG = load_robot_config(Path(__file__).resolve().parents[1] / 'config/crush.yaml')


def same_rotation(a, b):
    return abs(np.dot(a, b)) == pytest.approx(1)


def test_mounts_convert_robosub_ros2_frames_into_the_frd_body():
    origin = sim_position(CONFIG, -np.asarray(CONFIG['corner_link']))
    np.testing.assert_allclose(origin, CONFIG['base_link'])
    np.testing.assert_allclose(sim_position(CONFIG, np.asarray([0.1, 0.2, 0.3]) - CONFIG['corner_link']),
                               np.asarray(CONFIG['base_link']) + [0.1, -0.2, -0.3])
    # The VN100 is yawed pi; FRD axes keep that yaw. A Stonefish DVL looks along -z, so it rolls pi.
    np.testing.assert_allclose(rotation(sim_rpy([0, 0, math.pi])), rotation([0, 0, math.pi]), atol=1e-12)
    np.testing.assert_allclose(rotation(sim_rpy([0, 0, 0], True)), np.diag([1, -1, -1]), atol=1e-12)
    # A pitched-down camera pitches the same way in both conventions; yaw and pitch change sign.
    np.testing.assert_allclose(rotation(sim_rpy([0.1, 0.2, 0.3])), rotation([0.1, -0.2, -0.3]), atol=1e-12)


def test_robot_xml_mounts_sensors_at_their_static_transforms():
    sensors = {sensor.get('name'): sensor for sensor in build_robot(CONFIG).getroot().findall('robot/sensor')}
    for name, mount in CONFIG['sensors'].items():
        np.testing.assert_allclose(
            np.fromstring(sensors[name].find('origin').get('xyz'), sep=' '), sim_position(CONFIG, mount['position'])
        )
        assert float(sensors[name].get('rate')) == mount['rate']


def test_level_vehicle_reproduces_the_vectornav_driver_orientation():
    level = quaternion([0, 0, 0])
    mount = quaternion(sim_rpy(CONFIG['sensors']['imu']['rpy']))
    # q_ned2enu * yaw(pi): what task_planning reads from /vectornav/imu on a level, north-facing Crush.
    assert same_rotation(vectornav_orientation(level, mount), quaternion([math.pi, 0, math.pi / 2]))
    turned = quaternion([0, 0, 0.4])
    assert same_rotation(vectornav_orientation(turned, mount), quaternion([math.pi, 0, math.pi / 2 - 0.4]))


def test_depth_matches_the_arduino_and_median_filter():
    depth = Depth()
    assert depth.update(0.0) == 0.0
    one_metre = 1000 * 9.81
    assert depth.update(one_metre) == 0.0  # median of [0, 0, 1.01]
    assert depth.update(one_metre) == -1.01
    assert depth.update(1e6) == -1.01  # out of range repeats the last median
    assert depth.update(2 * one_metre) == -1.01
    assert Depth().update(1e6) is None


def test_gyro_publishes_counter_clockwise_rate_and_wrapped_integral():
    gyro = Gyro(100)
    rate, _ = gyro.update(0.5)  # FRD +z is clockwise seen from above
    assert rate == -0.5
    gyro = Gyro(100)
    for _ in range(300):
        _, yaw = gyro.update(-math.pi / 2)
    assert same_rotation(yaw, quaternion([0, 0, -math.pi / 2]))  # 270 degrees CCW wraps to -90
