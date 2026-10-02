"""Check real ROS services, generated geometry and orderly simulator shutdown."""

import argparse
import os
import re
import signal
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--graphical', action='store_true', help='Use an existing DISPLAY to check rendering as well')
    options = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='pool_ros_check_') as directory:
        log = Path(directory) / 'launch.log'
        environment = os.environ.copy()
        environment['ROS_LOG_DIR'] = str(Path(directory) / 'ros_logs')
        command = [
            'ros2',
            'launch',
            'robosub_simulation',
            'pool.launch.py',
            'spawn_robot:=false',
            f'headless:={"false" if options.graphical else "true"}',
            'pool_length:=50',
            'pool_width:=25',
            'pool_depth:=4',
            'window_res_x:=640',
            'window_res_y:=480',
            'rendering_quality:=low',
        ]
        rclpy.init()
        node = rclpy.create_node('pool_smoke_validation')
        with log.open('w') as output:
            process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT, env=environment)
            try:
                for service in ('enable_currents', 'disable_currents'):
                    client = node.create_client(Trigger, '/simulation/' + service)
                    assert client.wait_for_service(timeout_sec=25), f'Service missing: {service}'
                    future = client.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                    assert future.done() and future.result().success, f'Service call failed: {service}'
                assert ('stonefish_simulator', '/simulation') in node.get_node_names_and_namespaces()
                content = log.read_text()
                match = re.search(r'scenario: (/tmp/robosub_pool_[^\s]+/pool.scn)', content)
                assert match, content
                scenario = Path(match.group(1))
                assert scenario.exists(), 'Generated scenario disappeared during simulation'
                scene = ET.parse(scenario)
                floor = scene.find("static[@name='pool_floor']")
                assert floor.find('dimensions').get('xyz') == '50.6 25.6 0.3'
                assert floor.find('world_transform').get('xyz') == '0 0 4.15'
                parsers = list(Path(directory).rglob('stonefish_ros2_parser.log'))
                assert parsers, 'Parser log missing'
                assert not any('[ERROR]' in file.read_text() or '[CRITICAL]' in file.read_text() for file in parsers)
                assert process.poll() is None, 'Simulator exited early'
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
        print(content[-5000:])
        assert process.returncode == 0, f'Launch returned {process.returncode}'
        assert 'process has finished cleanly' in content, 'Simulator did not shut down cleanly'
        assert 'process has died' not in content, 'Simulator crashed during shutdown'
        assert not scenario.exists(), 'Generated scenario was not cleaned up on shutdown'
        print('PASS: ROS node, service calls, dimension overrides, parser log and shutdown cleanup')


if __name__ == '__main__':
    main()
