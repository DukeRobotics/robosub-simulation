"""Launch the Stonefish pool, with optional dimension overrides and headless physics."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from tempfile import TemporaryDirectory

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robosub_simulation.pool import load_config, write_scenario
from robosub_simulation.robot import add_robot_materials, build_robot, load_robot_config


def _launch_pool(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    overrides = {name: float(value(f'pool_{name}')) for name in ('length', 'width', 'depth') if value(f'pool_{name}')}
    config = load_config(Path(value('pool_config')), **overrides)
    rate = float(value('simulation_rate'))
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('simulation_rate must be a positive finite number')
    render_rate = float(value('render_rate'))
    if not math.isfinite(render_rate) or render_rate <= 0:
        raise ValueError('render_rate must be a positive finite number')
    headless = value('headless').lower()
    if headless not in ('true', 'false'):
        raise ValueError('headless must be true or false')
    for name in ('spawn_robot', 'enable_teleop', 'keyboard_teleop', 'keyboard_pulse_mode'):
        if value(name).lower() not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
    spawn_robot = value('spawn_robot').lower() == 'true'
    teleop = value('enable_teleop').lower() == 'true' and spawn_robot
    keyboard = value('keyboard_teleop').lower() == 'true' and teleop
    quality = value('rendering_quality')
    if quality not in ('low', 'medium', 'high'):
        raise ValueError('rendering_quality must be low, medium or high')
    for name in ('window_res_x', 'window_res_y'):
        if int(value(name)) <= 0:
            raise ValueError(f'{name} must be positive')

    temporary = TemporaryDirectory(prefix='robosub_pool_')
    scenario = write_scenario(config, Path(temporary.name) / 'pool.scn')
    robot_config = load_robot_config(value('robot_config')) if spawn_robot else None
    if spawn_robot:
        model = robot_config['model_data']
        dimensions = [high - low for low, high in zip(*model['bounds'], strict=True)]
        if any(
            available <= required + 0.1
            for available, required in zip((config.length, config.width, config.depth), dimensions, strict=True)
        ):
            raise ValueError('The robot needs at least 0.1 m clearance inside the pool')
        robot_config['water_density'] = config.water_density
        robot_path = Path(temporary.name) / 'crush.xml'
        build_robot(robot_config, cameras=headless == 'false', spawn=(0, 0, min(1.0, config.depth / 2))).write(
            robot_path, encoding='utf-8', xml_declaration=True
        )
        tree = ET.parse(scenario)
        add_robot_materials(tree.getroot())
        ET.SubElement(tree.getroot(), 'include', file=str(robot_path))
        ET.indent(tree, space='  ')
        tree.write(scenario, encoding='utf-8', xml_declaration=True)
    # Stonefish concatenates relative asset filenames onto this directory.
    data = str(Path(get_package_share_directory('robosub_simulation')) / 'scenarios') + '/'
    node = Node(
        package='robosub_stonefish',
        executable='pool_simulator',
        namespace='simulation',
        name='stonefish_simulator',
        parameters=[
            {
                'use_sim_time': False,
                'simulation_data': data,
                'scenario_desc': str(scenario),
                'simulation_rate': rate,
                'headless': headless == 'true',
                'window_res_x': int(value('window_res_x')),
                'window_res_y': int(value('window_res_y')),
                'rendering_quality': quality,
                'pool_length': float(config.length),
                'pool_width': float(config.width),
                'pool_depth': float(config.depth),
                'render_rate': render_rate,
                'robot_enabled': spawn_robot,
                'keyboard_teleop': keyboard,
                'keyboard_pulse_mode': value('keyboard_pulse_mode').lower() == 'true',
                'linear_speed': robot_config['teleop']['linear_speed'] if spawn_robot else 0.45,
                'angular_speed': robot_config['teleop']['angular_speed'] if spawn_robot else 0.5,
            }
        ],
        output='screen',
    )

    def cleanup(_context):
        temporary.cleanup()
        return []

    actions = [
        LogInfo(msg=f'Pool: {config.length:g} x {config.width:g} x {config.depth:g} m; scenario: {scenario}'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='Stonefish simulator exited')),
                ],
            )
        ),
        node,
    ]
    if teleop:
        mixer = Node(
            package='robosub_simulation',
            executable='teleop_mixer',
            namespace='simulation',
            name='crush_teleop',
            parameters=[{'robot_config': value('robot_config')}],
            output='screen',
        )
        actions.extend(
            [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=mixer,
                        on_exit=[
                            EmitEvent(event=Shutdown(reason='Thruster mixer exited')),
                        ],
                    )
                ),
                mixer,
            ]
        )
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('robosub_simulation'))
    arguments = [
        ('pool_config', str(share / 'config' / 'pool.yaml'), 'Pool YAML configuration'),
        ('pool_length', '', 'Override interior length in metres'),
        ('pool_width', '', 'Override interior width in metres'),
        ('pool_depth', '', 'Override water depth in metres'),
        ('headless', 'false', 'Run physics without a window, cameras or rendering'),
        ('spawn_robot', 'true', 'Spawn Crush with built-in Stonefish actuators and sensors'),
        ('enable_teleop', 'true', 'Start the prototype velocity controller and thruster mixer'),
        ('robot_config', str(share / 'config' / 'crush.yaml'), 'Provisional robot and teleop configuration'),
        ('keyboard_teleop', 'true', 'Enable graphical keyboard control of the robot'),
        ('keyboard_pulse_mode', 'false', 'Use expiring key pulses from the browser viewer'),
        ('simulation_rate', '100.0', 'Physics update rate in Hz'),
        ('window_res_x', '1280', 'Window width in pixels'),
        ('window_res_y', '720', 'Window height in pixels'),
        ('rendering_quality', 'low', 'Rendering quality: low, medium, high'),
        ('render_rate', '20.0', 'Maximum graphical frame rate in Hz'),
    ]
    return LaunchDescription(
        [
            *(
                DeclareLaunchArgument(name, default_value=default, description=description)
                for name, default, description in arguments
            ),
            OpaqueFunction(function=_launch_pool),
        ]
    )
