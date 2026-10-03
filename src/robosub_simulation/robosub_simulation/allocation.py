"""Allocate body forces and torques to the configured built-in Stonefish thrusters."""

import math

import numpy as np


class ThrusterAllocator:
    def __init__(self, config):
        columns = []
        names = []
        center_of_mass = np.asarray(config['model_data']['center_of_mass'], dtype=float)
        for thruster in config['thrusters']:
            _, pitch, yaw = thruster['rpy']
            direction = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch)])
            # Actuator poses use the CAD link frame; torques act about the centre of mass.
            lever = np.asarray(thruster['position'], dtype=float) - center_of_mass
            columns.append(np.concatenate((direction, np.cross(lever, direction))))
            names.append(thruster['name'])
        self.matrix = np.column_stack(columns)
        if not np.all(np.isfinite(self.matrix)) or np.linalg.matrix_rank(self.matrix) != 6:
            raise ValueError('Thruster layout must provide six independent motion axes')
        if len(set(names)) != len(names):
            raise ValueError('Thruster names must be unique')
        self.inverse = np.linalg.pinv(self.matrix)
        self.max_force = float(config['thruster']['max_force'])
        if not math.isfinite(self.max_force) or self.max_force <= 0:
            raise ValueError('max_force must be positive and finite')

    def allocate(self, wrench):
        wrench = np.asarray(wrench, dtype=float)
        if wrench.shape != (6,) or not np.all(np.isfinite(wrench)):
            raise ValueError('Wrench must contain six finite force/torque values')
        forces = self.inverse @ wrench
        # Scale the entire vector at saturation to preserve the requested wrench direction.
        forces /= max(1.0, float(np.max(np.abs(forces))) / self.max_force)
        return np.sign(forces) * np.sqrt(np.abs(forces) / self.max_force)


class VelocityController:
    def __init__(self, config):
        self.allocator = ThrusterAllocator(config)
        self.gains = np.asarray(config['teleop']['gains'], dtype=float)
        self.limits = np.asarray(config['teleop']['wrench_limits'], dtype=float)
        for values in (self.gains, self.limits):
            if values.shape != (6,) or not np.all(np.isfinite(values)) or np.any(values <= 0):
                raise ValueError('Controller gains and wrench limits must contain six positive finite numbers')

    def update(self, desired, measured):
        error = np.asarray(desired) - np.asarray(measured)
        if error.shape != (6,) or not np.all(np.isfinite(error)):
            raise ValueError('Velocities must contain six finite numbers')
        return self.allocator.allocate(np.clip(self.gains * error, -self.limits, self.limits))
