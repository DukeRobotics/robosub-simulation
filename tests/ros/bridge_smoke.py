"""Drive Crush through robosub-ros2 /controls/thruster_allocs and check the motion each thruster group produces."""

import signal
import subprocess
import time

import rclpy
from custom_msgs.msg import ThrusterAllocs
from nav_msgs.msg import Odometry
from stonefish_ros2.msg import ThrusterState

HORIZONTAL = [0, 3, 4, 7]
VERTICAL = [1, 2, 5, 6]


def main():
    rclpy.init()
    node = rclpy.create_node('bridge_smoke_validation')
    messages = {}
    node.create_subscription(Odometry, '/simulation/crush/ground_truth', lambda m: messages.update(gt=m), 10)
    node.create_subscription(ThrusterState, '/simulation/crush/thruster_state', lambda m: messages.update(state=m), 10)
    publisher = node.create_publisher(ThrusterAllocs, '/controls/thruster_allocs', 1)

    def spin(duration, allocs=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if allocs is not None:
                publisher.publish(ThrusterAllocs(allocs=allocs))
            rclpy.spin_once(node, timeout_sec=0.05)

    def velocity():
        v = messages['gt'].twist.twist.linear
        return v.x, v.y, v.z

    process = subprocess.Popen(
        ['ros2', 'launch', 'robosub_simulation', 'pool.launch.py', 'headless:=true', 'enable_teleop:=false'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        deadline = time.monotonic() + 30
        def ready():
            # Both hops discovered: this test -> sim_adapter -> Stonefish.
            return (
                'gt' in messages
                and publisher.get_subscription_count() > 0
                and node.count_subscribers('/simulation/crush/thruster_setpoints') > 0
            )

        while not ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert ready(), 'Simulator or sim_adapter did not start'
        spin(2)

        def push(indices):
            # Body-frame (forward/right/down) velocity change at thrust onset. Open loop, the off-centre CAD
            # centre of mass turns and tilts Crush and neutral buoyancy has no righting moment, so longer
            # pushes or world displacement do not isolate the thrust direction.
            before = velocity()
            spin(1.0, [0.4 if index in indices else 0.0 for index in range(8)])
            return [after - start for start, after in zip(before, velocity(), strict=True)]

        # Corner thrusters at +/-135 degrees: equal positive allocations push Crush backward.
        moved = push(HORIZONTAL)
        print(f'Horizontal +0.4: body velocity change {[round(v, 3) for v in moved]}', flush=True)
        assert moved[0] < -0.05 and max(abs(moved[1]), abs(moved[2])) < 0.25 * abs(moved[0]), (
            f'Expected reverse surge; rotor setpoints {list(messages["state"].setpoint)}'
        )
        # Thruster watchdogs stop the rotors once controls stops publishing.
        spin(1.0)
        assert all(abs(value) < 1e-8 for value in messages['state'].setpoint), 'Silence must cut thrust'
        spin(4)
        # Vertical thrusters push along forward/left/up -z: positive allocations dive.
        moved = push(VERTICAL)
        print(f'Vertical +0.4: body velocity change {[round(v, 3) for v in moved]}', flush=True)
        assert moved[2] > 0.05 and max(abs(moved[0]), abs(moved[1])) < 0.25 * moved[2], 'Expected a dive'
        assert len(messages['state'].setpoint) == 8
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=20)
        node.destroy_node()
        rclpy.shutdown()
    print('PASS: robosub-ros2 thruster allocations drive the simulated thrusters with the expected signs')


if __name__ == '__main__':
    main()
