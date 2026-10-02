"""Record ROS telemetry while a browser drives the robot, for an end-to-end check."""

import argparse
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray


def vector(twist):
    return [twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/robot_browser_telemetry.jsonl')
    parser.add_argument('--duration', type=float, default=80)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('browser_teleop_observer')
    with open(args.output, 'w', buffering=1) as stream:

        def record(topic, values):
            stream.write(json.dumps({'time': time.monotonic(), 'topic': topic, 'values': values}) + '\n')

        subscriptions = [
            node.create_subscription(
                Twist,
                '/simulation/crush/cmd_vel',
                lambda message: record('command', vector(message)),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                Odometry,
                '/simulation/crush/ground_truth',
                lambda message: record('velocity', vector(message.twist.twist)),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                Float64MultiArray,
                '/simulation/crush/thruster_setpoints',
                lambda message: record('setpoints', list(message.data)),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                Image,
                '/simulation/crush/camera/front/image_color',
                lambda message: record('camera', [message.width, message.height, len(message.data)]),
                qos_profile_sensor_data,
            ),
        ]
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
