import io
import math
from pathlib import Path

import numpy as np
import pytest
from robosub_simulation.robot import build_robot, load_robot_config, read_drivers, read_robosub_ros2
from PIL import Image
from robosub_simulation.sensors import (
    Depth,
    Gyro,
    jpeg,
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
    np.testing.assert_allclose(sim_position(CONFIG, [0, 0, 0]), CONFIG['base_link'])
    np.testing.assert_allclose(sim_position(CONFIG, [0.1, 0.2, 0.3]), np.asarray(CONFIG['base_link']) + [0.1, -0.2, -0.3])
    # The VN100 is yawed pi; FRD axes keep that yaw. A Stonefish DVL looks along -z, so it rolls pi.
    np.testing.assert_allclose(rotation(sim_rpy(rotation([0, 0, math.pi]))), rotation([0, 0, math.pi]), atol=1e-12)
    np.testing.assert_allclose(rotation(sim_rpy(np.eye(3), True)), np.diag([1, -1, -1]), atol=1e-12)
    # Roll keeps its sign between conventions; pitch and yaw flip.
    np.testing.assert_allclose(rotation(sim_rpy(rotation([0.1, 0.2, 0.3]))), rotation([0.1, -0.2, -0.3]), atol=1e-12)


def test_poses_come_from_the_robosub_ros2_configs():
    transforms, thrusters, cameras = read_robosub_ros2(CONFIG['robosub_ros2'], CONFIG['robot'])
    corner = transforms['corner_link']
    assert corner['frame_id'] == 'base_link'
    for name, mount in CONFIG['sensors'].items():
        item = transforms[mount['frame']]
        assert item['frame_id'] == 'corner_link'
        expected = [corner[axis] + item[axis] for axis in 'xyz']
        np.testing.assert_allclose(mount['position'], sim_position(CONFIG, expected), err_msg=name)
    assert [thruster['name'] for thruster in CONFIG['thrusters']] == [thruster['name'] for thruster in thrusters]
    assert set(CONFIG['cameras']) == set(cameras)
    drivers = read_drivers(CONFIG['robosub_ros2'], CONFIG['robot'])
    assert CONFIG['sensors']['imu']['frame'] == drivers['imu']['frame'] == 'imu'
    assert CONFIG['sensors']['imu']['rate'] == 20  # BO1 rateDivisor 40
    assert drivers['depth_topic'] == '/sensors/depth' and drivers['voltage_topic'] == '/sensors/voltage'
    assert len(drivers['imu']['angular_velocity']) == 9
    for name, camera in cameras.items():
        assert CONFIG['cameras'][name]['frame'] == camera['frame_id']
        assert CONFIG['cameras'][name]['topic'] == camera['topic']


def test_cameras_look_forward_and_down_with_image_up_toward_the_surface():
    # Stonefish renders along the camera's +z with -y as image up; the body is forward/right/down.
    front = rotation(CONFIG['cameras']['front']['orientation'])
    np.testing.assert_allclose(front[:, 2], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(-front[:, 1], [0, 0, -1], atol=1e-12)
    np.testing.assert_allclose(front[:, 0], [0, 1, 0], atol=1e-12)  # image right is vehicle right
    bottom = rotation(CONFIG['cameras']['bottom']['orientation'])
    np.testing.assert_allclose(bottom[:, 2], [0, 0, 1], atol=1e-12)
    np.testing.assert_allclose(-bottom[:, 1], [1, 0, 0], atol=1e-12)  # image up is vehicle forward


def test_camera_jpeg_preserves_colour_order():
    pixels = bytes(([255, 0, 0] * 32 + [0, 255, 0] * 32) * 16)  # rgb8: red left half, green right half
    decoded = np.asarray(Image.open(io.BytesIO(jpeg(64, 16, pixels))))
    assert decoded.shape == (16, 64, 3)
    assert decoded[8, 4, 0] > 200 and decoded[8, 4, 1] < 60
    assert decoded[8, 60, 1] > 200 and decoded[8, 60, 0] < 60


def test_robot_xml_mounts_sensors_at_their_static_transforms():
    sensors = {sensor.get('name'): sensor for sensor in build_robot(CONFIG).getroot().findall('robot/sensor')}
    mounts = {**CONFIG['sensors'], **{f'camera/{name}': mount for name, mount in CONFIG['cameras'].items()}}
    for name, mount in mounts.items():
        np.testing.assert_allclose(
            np.fromstring(sensors[name].find('origin').get('xyz'), sep=' '), mount['position']
        )
        assert float(sensors[name].get('rate')) == mount['rate']


def test_level_vehicle_reproduces_the_vectornav_driver_orientation():
    level = quaternion([0, 0, 0])
    mount = quaternion(CONFIG['sensors']['imu']['orientation'])
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
