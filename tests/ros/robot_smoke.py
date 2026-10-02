"""Exercise real vehicle physics, sensors, all six axes, and command-loss cutoff."""

import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Imu, Range
from std_srvs.srv import SetBool
from stonefish_ros2.msg import DVL, ThrusterState


def main():
    rclpy.init()
    node = rclpy.create_node('robot_smoke_validation')
    messages = {}
    subscriptions = []
    for name, kind in (
        ('ground_truth', Odometry),
        ('imu', Imu),
        ('pressure', FluidPressure),
        ('dvl', DVL),
        ('dvl/altitude', Range),
        ('thruster_state', ThrusterState),
    ):
        subscriptions.append(
            node.create_subscription(
                kind, '/simulation/crush/' + name, lambda message, key=name: messages.update({key: message}), 10
            )
        )
    publisher = node.create_publisher(Twist, '/simulation/crush/cmd_vel', 1)

    def spin(duration, command=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if command is not None:
                publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.025)

    def velocity():
        twist = messages['ground_truth'].twist.twist
        return np.array(
            [twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z]
        )

    with tempfile.TemporaryDirectory(prefix='robot_ros_check_') as directory:
        log = Path(directory) / 'launch.log'
        environment = os.environ.copy()
        environment['ROS_LOG_DIR'] = str(Path(directory) / 'ros_logs')
        with log.open('w') as stream:
            process = subprocess.Popen(
                [
                    'ros2',
                    'launch',
                    'robosub_simulation',
                    'pool.launch.py',
                    'headless:=true',
                    'keyboard_teleop:=false',
                ],
                stdout=stream,
                stderr=subprocess.STDOUT,
                env=environment,
            )
            try:
                deadline = time.monotonic() + 30
                while len(messages) < 6 and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    assert process.poll() is None, log.read_text()
                assert len(messages) == 6, f'Missing sensors: {messages.keys()}\n{log.read_text()}'
                position = messages['ground_truth'].pose.pose.position
                assert 0.8 < position.z < 1.2, f'Incorrect spawn depth: {position.z}'
                assert np.linalg.norm(velocity()) < 0.01, f'Neutral-buoyancy drift: {velocity()}'
                assert messages['pressure'].fluid_pressure > 5000
                assert 1.5 < messages['dvl/altitude'].range < 2.1, 'DVL should see the floor below the robot'
                assert len(messages['thruster_state'].setpoint) == 8

                for axis in range(6):
                    command = Twist()
                    target = command.linear if axis < 3 else command.angular
                    setattr(target, 'xyz'[axis % 3], 0.3 if axis < 3 else 0.35)
                    spin(3.0, command)
                    measured = velocity()
                    assert measured[axis] > 0.005, f'Axis {axis} did not move in the commanded direction: {measured}'
                    print(f'Axis {axis}: body velocity {measured.round(4).tolist()}', flush=True)
                    spin(2.0, Twist())

                command = Twist()
                command.linear.x = 0.3
                spin(1, command)
                assert any(abs(value) > 0.01 for value in messages['thruster_state'].setpoint)
                spin(0.8)
                assert all(abs(value) < 1e-8 for value in messages['thruster_state'].setpoint), (
                    'Lost command must cut thrust'
                )
                client = node.create_client(SetBool, '/simulation/crush/ground_truth/set_enabled')
                assert client.wait_for_service(timeout_sec=3)
                future = client.call_async(SetBool.Request(data=False))
                rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                assert future.done() and future.result().success
                spin(0.8, command)
                assert all(abs(value) < 1e-8 for value in messages['thruster_state'].setpoint), (
                    'Lost ground truth must cut thrust despite a live command'
                )
                parsers = list(Path(directory).rglob('stonefish_ros2_parser.log'))
                assert parsers and not any(
                    '[ERROR]' in file.read_text() or '[CRITICAL]' in file.read_text() for file in parsers
                )
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                node.destroy_node()
                rclpy.shutdown()
        content = log.read_text()
        print(content[-1800:])
        assert process.returncode == 0
        assert 'process has died' not in content, 'A simulator process crashed during shutdown'
        assert content.count('process has finished cleanly') == 2, 'Both simulator and mixer must stop cleanly'
        print('PASS: neutral spawn, real sensor messages, six-axis thruster motion, command timeout, clean shutdown')


if __name__ == '__main__':
    main()
