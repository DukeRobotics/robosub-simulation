"""Validate observer output from robot_teleop.cjs against actual ROS commands and motion."""

import argparse
import json
import math


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('file')
    args = parser.parse_args()
    with open(args.file) as stream:
        records = [json.loads(line) for line in stream]
    commands = [record for record in records if record['topic'] == 'command']
    velocities = [record for record in records if record['topic'] == 'velocity']
    assert commands and velocities, 'No robot command or ground-truth telemetry'
    for axis in range(6):
        for sign in (-1, 1):
            candidates = [
                record
                for record in commands
                if record['values'][axis] * sign > 0.1
                and all(abs(value) < 1e-8 for index, value in enumerate(record['values']) if index != axis)
            ]
            assert candidates, f'Browser key did not command axis {axis}, sign {sign}'
            first, last = candidates[0]['time'], candidates[-1]['time']
            samples = [record['values'][axis] * sign for record in velocities if first <= record['time'] <= last + 0.15]
            assert samples and max(samples) > 0.001, f'No physical motion on axis {axis}, sign {sign}'
    assert any(record['values'][0] > 0.1 and record['values'][1] > 0.1 for record in commands), 'Missing chord'
    last_motion = max(record['time'] for record in commands if any(abs(value) > 0.1 for value in record['values']))
    after = [record for record in commands if record['time'] > last_motion + 0.4]
    assert after and all(all(abs(value) < 1e-8 for value in record['values']) for record in after), (
        'Closing the browser while driving did not clear the keyboard command'
    )
    for record in velocities:
        assert all(math.isfinite(value) and abs(value) < 3 for value in record['values']), 'Unstable dynamics'
    setpoints = [record['values'] for record in records if record['topic'] == 'setpoints']
    assert setpoints and all(len(values) == 8 and all(abs(value) <= 1 for value in values) for values in setpoints)
    assert any(any(abs(value) > 0.01 for value in values) for values in setpoints), 'Thrusters never fired'
    cameras = [record['values'] for record in records if record['topic'] == 'camera']
    assert cameras and all(values[0:2] == [320, 240] and values[2] > 0 for values in cameras), 'Camera images missing'
    print(
        'PASS: browser keys reach ROS, all twelve directions move the robot, chording, input expiry, '
        'bounded thrusters, stable motion, real camera images'
    )


if __name__ == '__main__':
    main()
