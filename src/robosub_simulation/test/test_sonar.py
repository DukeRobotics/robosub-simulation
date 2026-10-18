import math
from pathlib import Path

import numpy as np
import pytest
from robosub_simulation.ping360 import Ping360, incidences
from robosub_simulation.robot import load_robot_config

CONFIG = load_robot_config(Path(__file__).resolve().parents[1] / 'config/crush.yaml')


@pytest.fixture(scope='module')
def ping():
    return Ping360(CONFIG['robosub_ros2'], CONFIG['robot'], seed=0)


def wall(distance, yaw):
    """Beam ranges to a flat wall `distance` ahead when the sonar is yawed counter-clockwise by `yaw`."""

    def beam(bearing):
        angle = yaw + bearing
        return (distance / math.cos(angle), abs(angle)) if math.cos(angle) > 0 else None

    return beam


@pytest.mark.parametrize('yaw', [-20, -10, 0, 10, 20])
def test_ninety_degree_sweep_returns_the_turn_that_squares_up_to_the_wall(ping, yaw):
    # sonar_tasks.rotate_to_normal yaws by +normal_angle, so a counter-clockwise offset needs a negative normal.
    result = ping.request(-45, 45, 7, wall(3.0, math.radians(yaw)))
    assert result['success'] and result['is_object']
    assert math.degrees(result['normal_angle']) == pytest.approx(-yaw, abs=2.5)
    x, _, _ = result['pose']
    assert x == pytest.approx(3.0 / math.cos(math.radians(yaw)), abs=0.3)


@pytest.mark.parametrize('half_span', [15, 25])
def test_narrow_sweeps_carry_robosub_ros2s_ninety_degree_assumption(ping, half_span):
    # robosub-ros2 robosub-2026: init_cartesian and the -pi/4 offset assume a 100-gradian sweep, so a
    # square-on wall reads (45 - half_span) degrees off. Reproduced as-is; update this test when it is fixed.
    result = ping.request(-half_span, half_span, 8, wall(3.0, 0.0))
    assert result['is_object']
    assert math.degrees(result['normal_angle']) == pytest.approx(45 - half_span, abs=3)


def test_open_water_and_invalid_requests(ping):
    assert ping.request(-45, 45, 7, lambda bearing: None) == {
        'success': True, 'message': '', 'is_object': False, 'pose': None, 'normal_angle': 0.0, 'angle_of_wall': 0.0
    }
    assert not ping.request(10, -10, 7, wall(3.0, 0.0))['success']
    assert not ping.request(-45, 45, -1, wall(3.0, 0.0))['success']


def test_sweep_order_and_timing_follow_the_driver(ping):
    seen = []
    ping.request(-15, 15, 5, lambda bearing: seen.append(bearing))
    # From the left (counter-clockwise) edge to the right, one gradian per step.
    assert seen[0] > 0 > seen[-1] and len(seen) == ping.gradians(15) - ping.gradians(-15) - 1
    assert ping.step_duration() == pytest.approx(2 * 5 / 1482 + 0.02)


def test_incidence_is_zero_head_on_and_grows_toward_grazing():
    angles = np.radians(np.arange(-180, 180, 0.9))
    ranges = np.where(np.cos(angles) > 0.05, 3.0 / np.maximum(np.cos(angles), 0.05), 60.0)
    result = incidences(angles, ranges)
    ahead = np.argmin(np.abs(angles))
    assert result[ahead] == pytest.approx(0, abs=0.02)
    oblique = np.argmin(np.abs(angles - math.radians(40)))
    assert result[oblique] == pytest.approx(math.radians(40), abs=0.02)
