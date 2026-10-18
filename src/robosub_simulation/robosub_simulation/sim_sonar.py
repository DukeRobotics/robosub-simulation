"""Serve robosub-ros2's /sonar/request from simulated Ping360 sweeps of the pool."""

import math
import time

import numpy as np
import rclpy
from custom_msgs.srv import SonarSweepRequest
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from robosub_simulation.ping360 import Ping360, incidences
from robosub_simulation.robot import load_robot_config


class SimSonar(Node):
    """A separate node, like sonar.py: a sweep blocks for its real duration without stalling other sensors."""

    def __init__(self):
        super().__init__('sim_sonar')
        config = load_robot_config(self.declare_parameter('robot_config', '').value)
        self.ping = Ping360(config['robosub_ros2'], config['robot'])
        self.scan = None
        self.create_subscription(
            LaserScan, '/simulation/crush/sonar', self.scan_received, 1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_service(
            SonarSweepRequest, '/sonar/request', self.requested, callback_group=MutuallyExclusiveCallbackGroup()
        )

    def scan_received(self, message):
        # Stonefish rays sweep from the sonar's +x towards +y (right), the reverse of counter-clockwise bearings.
        ranges = np.asarray(message.ranges, dtype=float)
        angles = message.angle_min + message.angle_increment * np.arange(len(ranges))
        hit = np.isfinite(ranges) & (ranges < message.range_max)
        self.scan = (message.angle_min, message.angle_increment, ranges, hit, incidences(angles, ranges))

    def beam(self, bearing):
        # The head rotates and pings each angle in turn, so the robot can move during a sweep.
        time.sleep(self.ping.step_duration())
        if self.scan is None:
            return None
        angle_min, increment, ranges, hit, incidence = self.scan
        index = round((-bearing - angle_min) / increment) % len(ranges)
        return (ranges[index], incidence[index]) if hit[index] else None

    def requested(self, request, response):
        result = self.ping.request(request.start_angle, request.end_angle, request.distance_of_scan, self.beam)
        response.success = result['success']
        response.message = result['message']
        response.is_object = result['is_object']
        if result['pose'] is not None:
            x, y, yaw = result['pose']
            response.pose.header.stamp = self.get_clock().now().to_msg()
            response.pose.header.frame_id = 'robot_sonar'
            response.pose.pose.position.x, response.pose.pose.position.y = x, y
            response.pose.pose.orientation.z, response.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            response.normal_angle = result['normal_angle']
            response.angle_of_wall = result['angle_of_wall']
        return response


def main():
    rclpy.init()
    node = SimSonar()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
