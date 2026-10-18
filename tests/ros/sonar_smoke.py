"""Turn Crush in a small pool and check the simulated Ping360 wall normal against ground truth."""

import math
import signal
import subprocess
import time

import rclpy
from custom_msgs.srv import SonarSweepRequest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def main():
    rclpy.init()
    node = rclpy.create_node('sonar_smoke_validation')
    messages = {}
    node.create_subscription(Odometry, '/simulation/crush/ground_truth', lambda m: messages.update(gt=m), 10)
    command = node.create_publisher(Twist, '/simulation/crush/cmd_vel', 1)
    sonar = node.create_client(SonarSweepRequest, '/sonar/request')

    def spin(duration, twist=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if twist is not None:
                command.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.05)

    def yaw():
        # Ground truth is NED; counter-clockwise from above is negative NED yaw.
        q = messages['gt'].pose.pose.orientation
        return -math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def sweep(half_span):
        started = time.monotonic()
        future = sonar.call_async(SonarSweepRequest.Request(start_angle=-float(half_span),
                                                            end_angle=float(half_span), distance_of_scan=7.0))
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        assert future.done() and future.result().success, 'Sonar request failed'
        return future.result(), time.monotonic() - started

    # Walls 4 m from the pool centre, where Crush spawns facing +x.
    process = subprocess.Popen(
        ['ros2', 'launch', 'robosub_simulation', 'pool.launch.py', 'headless:=true', 'keyboard_teleop:=false',
         'pool_length:=8.0', 'pool_width:=8.0'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        deadline = time.monotonic() + 30
        while ('gt' not in messages or not sonar.service_is_ready() or command.get_subscription_count() == 0) \
                and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert 'gt' in messages and sonar.service_is_ready(), 'Simulator or sim_sonar did not start'
        spin(2)
        for target in (0.0, math.radians(15), math.radians(-15)):
            turn = Twist()
            # Teleop takes forward/right/down rates: negative z turns counter-clockwise.
            turn.angular.z = -0.15 if target > yaw() else 0.15
            deadline = time.monotonic() + 20
            # The prototype teleop coasts after release, so stop short of the target.
            while abs(target - yaw()) > math.radians(6) and time.monotonic() < deadline:
                spin(0.05, turn)
            spin(8, Twist())
            before = yaw()
            result, duration = sweep(45)
            truth = (before + yaw()) / 2  # The sweep takes seconds; use the mean heading.
            normal = math.degrees(result.normal_angle)
            print(f'Yaw {math.degrees(before):+.1f} -> {math.degrees(yaw()):+.1f} deg: normal {normal:+.1f} deg'
                  f' (expected {-math.degrees(truth):+.1f}), wall at x={result.pose.pose.position.x:.2f} m,'
                  f' sweep {duration:.1f} s', flush=True)
            assert result.is_object and abs(normal + math.degrees(truth)) < 3, 'Wall normal disagrees with ground truth'
            assert 2.0 < duration < 6.0, 'A 90 degree, 7 m sweep should take as long as the real Ping360'
        result, _ = sweep(25)
        print(f'+/-25 deg sweep: normal {math.degrees(result.normal_angle):+.1f} deg '
              '(robosub-ros2 reads narrow sweeps offset by 45 - 25 = 20 deg)', flush=True)
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=20)
        node.destroy_node()
        rclpy.shutdown()
    print('PASS: simulated Ping360 sweeps give robosub-ros2 wall normals that match ground truth')


if __name__ == '__main__':
    main()
