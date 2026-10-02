"""Convert body-frame velocity commands into built-in Stonefish thruster setpoints."""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from robosub_simulation.allocation import VelocityController
from robosub_simulation.robot import load_robot_config


def components(twist):
    return np.array([twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z])


class TeleopMixer(Node):
    def __init__(self):
        super().__init__('crush_teleop')
        config = load_robot_config(self.declare_parameter('robot_config', '').value)
        self.controller = VelocityController(config)
        self.command_timeout = float(config['teleop']['command_timeout'])
        self.odometry_timeout = float(config['teleop']['odometry_timeout'])
        if not all(math.isfinite(value) and value > 0 for value in (self.command_timeout, self.odometry_timeout)):
            raise ValueError('Timeouts must be positive and finite')
        self.speed_limits = np.array([config['teleop']['linear_speed']] * 3 + [config['teleop']['angular_speed']] * 3)
        if not np.all(np.isfinite(self.speed_limits)) or np.any(self.speed_limits <= 0):
            raise ValueError('Speed limits must be positive and finite')
        self.command = np.zeros(6)
        self.velocity = np.zeros(6)
        self.command_time = self.odometry_time = -math.inf
        self.publisher = self.create_publisher(Float64MultiArray, '/simulation/crush/thruster_setpoints', 1)
        self.create_subscription(Twist, '/simulation/crush/cmd_vel', self.command_received, 1)
        self.create_subscription(Odometry, '/simulation/crush/ground_truth', self.odometry_received, 1)
        self.create_timer(1 / 30, self.update)

    def command_received(self, message):
        values = components(message)
        if np.all(np.isfinite(values)):
            self.command = np.clip(values, -self.speed_limits, self.speed_limits)
            self.command_time = time.monotonic()
        else:
            self.command_time = -math.inf

    def odometry_received(self, message):
        # Stonefish Odometry reports twist in the sensor's local FRD axes.
        values = components(message.twist.twist)
        if np.all(np.isfinite(values)):
            self.velocity = values
            self.odometry_time = time.monotonic()
        else:
            self.odometry_time = -math.inf

    def update(self):
        now = time.monotonic()
        if now - self.command_time > self.command_timeout or now - self.odometry_time > self.odometry_timeout:
            setpoints = np.zeros(len(self.controller.allocator.matrix.T))
        else:
            setpoints = self.controller.update(self.command, self.velocity)
        self.publisher.publish(Float64MultiArray(data=setpoints.tolist()))


def main():
    rclpy.init()
    node = TeleopMixer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Float64MultiArray(data=[0.0] * len(node.controller.allocator.matrix.T)))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
