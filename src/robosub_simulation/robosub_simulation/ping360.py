"""Ping360 sweeps synthesized from Stonefish ray ranges and processed by robosub-ros2's own sonar code.

Request handling mirrors onboard/src/sonar/sonar/sonar.py; its tuning constants are read from that file and
the denoising, segmentation and wall fit are imported from sonar_object_detection.py, so the simulated
sonar answers with the team's real processing.
"""

import ast
import math
import sys
from pathlib import Path

import numpy as np
import yaml

# sonar_utils.py
SPEED_OF_SOUND_IN_WATER = 1482.0  # m/s
SAMPLE_PERIOD_TICK_DURATION = 25e-9  # s
TRANSFORMATION_ANGLE = math.pi / 4
GRADIANS_PER_CIRCLE = 400
# ponytail: echo model from Ping360 behaviour, not measured; tune against recorded sweeps of a pool wall.
NOISE_FLOOR = 8
ECHO_MIN = 90
# Head rotation, serial transfer and processing per angle, on top of the round trip.
STEP_OVERHEAD = 0.02  # s


def driver_constants(root):
    """Literal class constants of robosub-ros2's Sonar node, without importing its hardware dependencies."""
    tree = ast.parse((Path(root) / 'onboard/src/sonar/sonar/sonar.py').read_text())
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == 'Sonar')
    constants = {}
    for item in node.body:
        if isinstance(item, ast.Assign) and len(item.targets) == 1 and isinstance(item.targets[0], ast.Name):
            try:
                constants[item.targets[0].id] = ast.literal_eval(item.value)
            except ValueError:
                pass
    return constants


def sweep_center(root):
    """robosub-ros2's sweep_center_angle(rows) from sonar_utils.py, or its older fixed pi/4 centre.

    Only that function is loaded: the module imports ROS packages the simulator does not need.
    """
    tree = ast.parse((Path(root) / 'onboard/src/sonar/sonar/sonar_utils.py').read_text())
    wanted = {'sweep_center_angle', 'MAX_SWEEP_ROWS'}
    body = [
        item for item in tree.body
        if (isinstance(item, ast.FunctionDef) and item.name in wanted)
        or (isinstance(item, ast.Assign) and any(getattr(t, 'id', None) in wanted for t in item.targets))
    ]
    if not any(isinstance(item, ast.FunctionDef) for item in body):
        return lambda rows: TRANSFORMATION_ANGLE
    namespace = {'np': np}
    exec(compile(ast.Module(body=body, type_ignores=[]), 'sonar_utils.py', 'exec'), namespace)  # noqa: S102
    return namespace['sweep_center_angle']


def incidences(angles, ranges):
    """Angle between each ray and the surface it hits, from the neighbouring hits of a 360-degree scan."""
    points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
    tangent = np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)
    rays = np.column_stack((np.cos(angles), np.sin(angles)))
    along = np.abs(np.sum(tangent * rays, axis=1)) / np.maximum(np.linalg.norm(tangent, axis=1), 1e-9)
    return np.arcsin(np.clip(along, 0.0, 1.0))


class Ping360:
    """robosub-ros2 sonar.py request handling with simulated sweeps."""

    def __init__(self, root, robot, seed=None):
        root = Path(root)
        self.constants = driver_constants(root)
        config = yaml.safe_load((root / 'onboard/src/sonar/config' / f'{robot}.yaml').read_text())
        self.center_gradians = config['center_gradians']
        self.ccw = config['increase_ccw']
        sys.path.insert(0, str(root / 'onboard/src/sonar'))
        from sonar import sonar_object_detection

        self.detection = sonar_object_detection
        self.center = sweep_center(root)
        self.samples = self.constants['DEFAULT_NUMER_OF_SAMPLES']
        self.random = np.random.default_rng(seed)
        self.set_range(self.constants['DEFAULT_RANGE'])

    def set_range(self, sonar_range):
        """sonar.py set_new_range with sonar_utils range_to_period and range_to_transmit."""
        self.range = sonar_range
        self.period = round(
            2 * sonar_range / (self.samples * SPEED_OF_SOUND_IN_WATER * SAMPLE_PERIOD_TICK_DURATION)
        )
        self.metres_per_sample = SPEED_OF_SOUND_IN_WATER * self.period * SAMPLE_PERIOD_TICK_DURATION / 2
        transmit = round(max(5, min(500, max(self.period / 40, 8000 * sonar_range / SPEED_OF_SOUND_IN_WATER))))
        self.pulse_samples = transmit * 1e-6 / (self.period * SAMPLE_PERIOD_TICK_DURATION)

    def step_duration(self):
        return 2 * self.range / SPEED_OF_SOUND_IN_WATER + STEP_OVERHEAD

    def gradians(self, degrees):
        """sonar_utils degrees_to_centered_gradians."""
        return int(degrees * GRADIANS_PER_CIRCLE / 360 * (1 if self.ccw else -1) + self.center_gradians)

    def bearing(self, gradians):
        """sonar_utils centered_gradians_to_radians: counter-clockwise from the sonar's forward axis."""
        return (gradians - self.center_gradians) * 2 * math.pi / GRADIANS_PER_CIRCLE * (1 if self.ccw else -1)

    def echo(self, hit):
        """One Ping360 intensity row: speckled noise floor plus a wall echo weaker at grazing incidence."""
        row = self.random.integers(0, NOISE_FLOOR + 1, self.samples).astype(float)
        if hit is not None:
            distance, incidence = hit
            centre = distance / self.metres_per_sample - 0.5  # sonar_utils get_distance_of_sample
            if centre < self.samples:
                peak = ECHO_MIN + (255 - ECHO_MIN) * max(math.cos(incidence), 0.0)
                sigma = max(self.pulse_samples / 2, 1.0)
                row += peak * np.exp(-0.5 * ((np.arange(self.samples) - centre) / sigma) ** 2)
        row = np.clip(row, 0, 255).astype(int)
        row[: self.constants['FILTER_INDEX']] = 0
        return row

    def request(self, start_degrees, end_degrees, distance, beam):
        """sonar.py perform_sonar_request. `beam(bearing)` returns (range, incidence) or None for no hit."""
        result = {'success': False, 'message': '', 'is_object': False, 'pose': None, 'normal_angle': 0.0,
                  'angle_of_wall': 0.0}
        if start_degrees > end_degrees:
            result['message'] = f'Start angle {start_degrees} must be less than or equal to end angle {end_degrees}.'
            return result
        left, right = self.gradians(start_degrees), self.gradians(end_degrees)
        if left < 0 or right < 0 or right > GRADIANS_PER_CIRCLE or distance < 0:
            return result
        if distance != self.range:
            self.set_range(distance)
        # get_sweep: from end to start so the top edge of the sweep is the robot's left side.
        sweep = np.vstack(
            [self.echo(beam(self.bearing(angle))) for angle in range(right, left + 1, -1 if self.ccw else 1)]
        )
        result['success'] = True
        found = self.process(sweep)
        if found is not None:
            result.update(found, is_object=True)
        return result

    def process(self, sweep):
        """sonar.py get_xy_of_object_in_sweep with robosub-ros2's processing classes and constants."""
        c = self.constants
        denoiser = self.detection.SonarDenoiser(sweep)
        denoiser.wall_block().percentile_filter().fourier_signal_processing(
            threshold=c['FOURIER_THRESHOLD'],
        ).init_cartesian().normalize().blur(factor=c['BLUR_FACTOR'], cutoff=c['BLUR_CUTOFF'])
        segmentation = self.detection.ClusteredSonarSegmentation(
            denoiser.cartesian,
            eps=c['DBSCAN_EPS'],
            min_samples=c['DBSCAN_MIN_SAMPLES'],
            wall_fit_inlier_threshold=c['WALL_FIT_INLIER_THRESHOLD'],
            wall_fit_ransac_iterations=c['WALL_FIT_RANSAC_ITERATIONS'],
        )
        segment = segmentation.get_most_wall_like_segment(
            c['MIN_WALL_ELONGATION'], c['MIN_WALL_SPAN_METERS'] / self.metres_per_sample
        )
        if segment is None:
            return None
        x_index, y_index = segment.get_average_coordinate_of_points()
        normal = segment.ortho_regression.unit_normal
        center = float(self.center(sweep.shape[0]))
        return {
            'pose': self.robot_position(x_index, y_index, center),
            'normal_angle': float(np.arctan2(normal[1], normal[0]) - center),
            'angle_of_wall': float(np.arctan(segment.ortho_regression.slope) + center),
        }

    def robot_position(self, x_index, y_index, center):
        """sonar_utils to_robot_position as (x, y, yaw)."""
        x, y = ((index + 0.5) * self.metres_per_sample for index in (x_index, y_index))
        sign = -1.0 if self.constants['NEGATE_POSE'] else 1.0
        cos, sin = math.cos(center), math.sin(center)
        x, y = (x * cos + y * sin) * sign, (-x * sin + y * cos) * sign
        return x, y, math.atan2(y, x)
