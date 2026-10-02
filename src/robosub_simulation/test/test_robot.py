import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from robosub_simulation.allocation import ThrusterAllocator, VelocityController
from robosub_simulation.robot import build_robot, load_robot_config

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = load_robot_config(PACKAGE / 'config/crush.yaml')


@pytest.mark.parametrize('axis', range(6))
@pytest.mark.parametrize('sign', [-1, 1])
def test_all_axes_produce_the_requested_wrench_without_cross_axis_leakage(axis, sign):
    allocator = ThrusterAllocator(CONFIG)
    target = np.zeros(6)
    target[axis] = sign
    setpoints = allocator.allocate(target)
    forces = setpoints * np.abs(setpoints) * allocator.max_force
    np.testing.assert_allclose(allocator.matrix @ forces, target, atol=1e-10)
    assert np.all(np.abs(setpoints) <= 1)


def test_saturation_preserves_wrench_direction():
    allocator = ThrusterAllocator(CONFIG)
    target = np.array([200, -100, 80, 40, -70, 20])
    setpoints = allocator.allocate(target)
    result = allocator.matrix @ (setpoints * np.abs(setpoints) * allocator.max_force)
    np.testing.assert_allclose(result / np.linalg.norm(result), target / np.linalg.norm(target), atol=1e-10)
    assert np.max(np.abs(setpoints)) == pytest.approx(1)


def test_zero_command_brakes_velocity_and_stationary_robot_has_zero_thrust():
    controller = VelocityController(CONFIG)
    np.testing.assert_array_equal(controller.update(np.zeros(6), np.zeros(6)), np.zeros(8))
    speed = np.array([0.2, -0.1, 0.1, 0.1, -0.2, 0.2])
    setpoints = controller.update(np.zeros(6), speed)
    force = controller.allocator.matrix @ (setpoints * np.abs(setpoints) * controller.allocator.max_force)
    assert np.all(force * speed < 0)


@pytest.mark.parametrize('wrench', [[math.nan] * 6, [math.inf] * 6, [0] * 5])
def test_nonfinite_or_wrong_size_commands_are_rejected(wrench):
    with pytest.raises(ValueError):
        ThrusterAllocator(CONFIG).allocate(wrench)


def test_principal_inertia_preserves_the_full_cad_tensor():
    roll, pitch, yaw = CONFIG['model_data']['principal_rpy']
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    np.testing.assert_allclose(
        rotation @ np.diag(CONFIG['model_data']['principal_inertia']) @ rotation.T,
        CONFIG['model_data']['inertia_tensor'],
        atol=1e-12,
    )
    assert np.linalg.det(rotation) == pytest.approx(1)


def test_buoyancy_proxy_preserves_mass_and_is_neutral_in_default_water():
    root = build_robot(CONFIG).getroot()
    hull = root.find('robot/base_link/external_part')
    buoyancy = root.find('robot/base_link/internal_part')
    mass = CONFIG['model_data']['mass']
    assert float(hull.find('mass').get('value')) + float(buoyancy.find('mass').get('value')) == pytest.approx(mass)
    radius = float(buoyancy.find('dimensions').get('radius'))
    assert 4 / 3 * math.pi * radius**3 * CONFIG['water_density'] == pytest.approx(mass)
    assert hull.get('buoyant') == 'false'


def test_thruster_xml_and_allocator_share_order_geometry_and_force_model():
    root = build_robot(CONFIG).getroot()
    actuators = root.findall('robot/actuator')
    assert [actuator.get('name') for actuator in actuators] == [thruster['name'] for thruster in CONFIG['thrusters']]
    for actuator, definition in zip(actuators, CONFIG['thrusters'], strict=True):
        assert actuator.get('type') == 'thruster'
        np.testing.assert_allclose(
            np.fromstring(actuator.find('origin').get('xyz'), sep=' '),
            np.array(CONFIG['model_data']['center_of_mass']) + definition['offset'],
        )
        coefficient = float(actuator.find('thrust_model/thrust_coeff').get('value'))
        assert coefficient * CONFIG['thruster']['max_omega'] ** 2 == pytest.approx(CONFIG['thruster']['max_force'])
        assert float(actuator.find('watchdog').get('timeout')) > 0


def test_headless_robot_keeps_scalar_sensors_and_omits_the_camera():
    sensors = build_robot(CONFIG, cameras=False).getroot().findall('robot/sensor')
    assert {sensor.get('type') for sensor in sensors} == {'odometry', 'imu', 'pressure', 'dvl'}
    assert all(sensor.find('ros_publisher') is not None for sensor in sensors)
    for mesh in build_robot(CONFIG).getroot().iter('mesh'):
        assert (PACKAGE / 'scenarios' / mesh.get('filename')).is_file()


def test_checked_in_robot_xml_matches_the_config():
    expected = build_robot(CONFIG).getroot()
    actual = ET.parse(
        PACKAGE / 'scenarios/crush.xml', parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    ).getroot()
    assert ET.tostring(actual) == ET.tostring(expected)
