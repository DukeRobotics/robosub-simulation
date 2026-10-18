"""Build the provisional Crush Stonefish robot from CAD-derived properties."""

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

from robosub_simulation.pool import _vector
from robosub_simulation.sensors import rotation, sim_camera_rpy, sim_position, sim_rpy


# VN100 binary outputs run at the 800 Hz IMU rate divided by rateDivisor.
VN100_RATE = 800.0


def read_robosub_ros2(root, robot):
    """Static transforms by child frame, thrusters and USB cameras from a robosub-ros2 checkout."""

    def read(package):
        return yaml.safe_load((Path(root) / 'onboard/src' / package / 'config' / f'{robot}.yaml').read_text())

    transforms = {item['child_frame_id']: item for item in read('static_transforms')['transforms'].values()}
    return transforms, read('controls')['thrusters'], read('cv')['usb_cameras']


def read_drivers(root, robot):
    """VectorNav driver parameters and peripheral sensor topics from a robosub-ros2 checkout."""
    vectornav = yaml.safe_load((Path(root) / 'onboard/src/vectornav/vectornav/config/vectornav.yaml').read_text())
    device, output = vectornav['vectornav']['ros__parameters'], vectornav['vn_sensor_msgs']['ros__parameters']
    if output['use_enu']:
        raise ValueError('sim_adapter reproduces the VN100 driver with use_enu: false only')
    offboard = yaml.safe_load((Path(root) / 'onboard/src/offboard_comms/config' / f'{robot}.yaml').read_text())
    topics = {sensor['type']: sensor['topic'] for sensor in offboard['arduino']['peripheral']['sensors']}
    return {
        'imu': {
            'frame': device['frame_id'],
            'rate': VN100_RATE / device['BO1']['rateDivisor'],
            **{name: output[f'{name}_covariance'] for name in ('orientation', 'angular_velocity', 'linear_acceleration')},
        },
        'depth_topic': topics['pressure'],
        'voltage_topic': topics['voltage'],
    }


def resolve(transforms, frame):
    """Position and rotation of a static-transform frame in base_link (forward/left/up)."""
    if frame == 'base_link':
        return np.zeros(3), np.eye(3)
    item = transforms[frame]
    position, matrix = resolve(transforms, item['frame_id'])
    local = rotation([item['roll'], item['pitch'], item['yaw']])
    return position + matrix @ [item['x'], item['y'], item['z']], matrix @ local


def load_robot_config(path):
    """Sim physics from `path`; frames, thrusters and cameras from the robosub-ros2 checkout it names.

    Mounts gain `position` and `orientation` in the Stonefish link frame (forward/right/down).
    """
    path = Path(path)
    config = yaml.safe_load(path.read_text())
    config['model_data'] = yaml.safe_load((path.parent / config['model']).read_text())
    root = path.parent / config['robosub_ros2']
    config['robosub_ros2'] = str(root)
    transforms, thrusters, cameras = read_robosub_ros2(root, config['robot'])
    config['drivers'] = read_drivers(root, config['robot'])
    vectornav = config['drivers']['imu']
    config['sensors']['imu'] = {'frame': vectornav['frame'], 'rate': vectornav['rate']}

    def mount(frame, orient=sim_rpy, **values):
        position, matrix = resolve(transforms, frame)
        return {'frame': frame, 'position': sim_position(config, position), 'orientation': orient(matrix), **values}

    config['sensors'] = {
        name: mount(
            sensor['frame'],
            # A Stonefish DVL looks along its -z, so it takes the robosub-ros2 frame itself.
            (lambda matrix: sim_rpy(matrix, reports_frame_axes=True)) if name == 'dvl' else sim_rpy,
            **{key: value for key, value in sensor.items() if key != 'frame'},
        )
        for name, sensor in config['sensors'].items()
    }
    config['cameras'] = {
        name: mount(
            camera['frame_id'],
            sim_camera_rpy,
            rate=config['camera_rate'],
            resolution=[camera['img_size']['width'], camera['img_size']['height']],
            fov=float(camera['horizontal_fov']),
            topic=camera['topic'],
        )
        for name, camera in cameras.items()
    }
    # controls config: corner_link poses with degree angles; force along each rotated +x, in allocation order.
    corner, corner_matrix = resolve(transforms, 'corner_link')
    config['thrusters'] = [
        {
            'name': thruster['name'],
            'position': sim_position(config, corner + corner_matrix @ thruster['pos']),
            'orientation': sim_rpy(corner_matrix @ rotation(np.radians(thruster['rpy']))),
        }
        for thruster in thrusters
    ]
    return config


def build_robot(config, *, cameras=True, spawn=(0.0, 0.0, 1.0)):
    model = config['model_data']
    root = ET.Element('scenario')
    root.append(ET.Comment('Provisional six-axis Crush model. Metres; world NED, body forward/right/down.'))
    robot = ET.SubElement(root, 'robot', name='crush', fixed='false', self_collisions='false')
    body = ET.SubElement(robot, 'base_link', name='base_link', type='compound', physics='submerged')
    hull = ET.SubElement(body, 'external_part', name='cad_hull', type='model', physics='submerged', buoyant='false')
    for kind, mesh in (('physical', 'crush_collision.obj'), ('visual', 'crush_visual.obj')):
        part = ET.SubElement(hull, kind)
        ET.SubElement(part, 'mesh', filename='../meshes/' + mesh, scale='1.0')
        ET.SubElement(part, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.SubElement(hull, 'material', name='robot_aluminium')
    ET.SubElement(hull, 'look', name='robot_hull')
    # The proxy contributes a tiny mass. Subtract that same mass from the CAD hull.
    proxy_mass = 1e-6
    ET.SubElement(hull, 'mass', value=str(model['mass'] - proxy_mass))
    ET.SubElement(hull, 'inertia', xyz=_vector(model['principal_inertia']))
    ET.SubElement(hull, 'cg', xyz=_vector(model['center_of_mass']), rpy=_vector(model['principal_rpy']))
    ET.SubElement(
        hull,
        'hydrodynamics',
        quadratic_drag=_vector(config['quadratic_drag']),
        viscous_drag=_vector(config['viscous_drag']),
    )
    ET.SubElement(hull, 'compound_transform', xyz='0 0 0', rpy='0 0 0')
    buoyancy = ET.SubElement(
        body, 'internal_part', name='buoyancy_proxy', type='sphere', physics='submerged', buoyant='true'
    )
    radius = (3 * model['mass'] / (4 * math.pi * config['water_density'])) ** (1 / 3)
    ET.SubElement(buoyancy, 'dimensions', radius=str(radius))
    ET.SubElement(buoyancy, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.SubElement(buoyancy, 'material', name='robot_neutral')
    ET.SubElement(buoyancy, 'look', name='robot_hull')
    ET.SubElement(buoyancy, 'mass', value=str(proxy_mass))
    ET.SubElement(buoyancy, 'compound_transform', xyz=_vector(model['center_of_mass']), rpy='0 0 0')

    specifications = config['thruster']
    for definition in config['thrusters']:
        thruster = ET.SubElement(robot, 'actuator', name=definition['name'], type='thruster')
        ET.SubElement(thruster, 'link', name='base_link')
        ET.SubElement(thruster, 'origin', xyz=_vector(definition['position']), rpy=_vector(definition['orientation']))
        ET.SubElement(thruster, 'watchdog', timeout=str(specifications['watchdog']))
        ET.SubElement(
            thruster,
            'specs',
            max_setpoint=str(specifications['max_omega']),
            inverted_setpoint='false',
            normalized_setpoint='true',
        )
        propeller = ET.SubElement(thruster, 'propeller', diameter='0.07', right='true')
        ET.SubElement(propeller, 'mesh', filename='../meshes/propeller.obj', scale='1.0')
        ET.SubElement(propeller, 'material', name='robot_aluminium')
        ET.SubElement(propeller, 'look', name='robot_propeller')
        dynamics = ET.SubElement(thruster, 'rotor_dynamics', type='first_order')
        ET.SubElement(dynamics, 'time_constant', value=str(specifications['time_constant']))
        thrust = ET.SubElement(thruster, 'thrust_model', type='quadratic')
        ET.SubElement(thrust, 'thrust_coeff', value=str(specifications['max_force'] / specifications['max_omega'] ** 2))

    def sensor(name, kind, rate, position=None, rpy=(0, 0, 0)):
        element = ET.SubElement(robot, 'sensor', name=name, type=kind, rate=format(rate, 'g'))
        ET.SubElement(element, 'link', name='base_link')
        position = model['center_of_mass'] if position is None else position
        ET.SubElement(element, 'origin', xyz=_vector(position), rpy=_vector(rpy))
        ET.SubElement(element, 'ros_publisher', topic=f'/simulation/crush/{name}')
        ET.SubElement(element, 'ros_service', set_enabled=f'/simulation/crush/{name}/set_enabled')
        return element

    def mounted(name, kind, mount):
        """Sensor at its robosub-ros2 static transform."""
        return sensor(name, kind, mount['rate'], mount['position'], mount['orientation'])

    sensor('ground_truth', 'odometry', 30)
    imu = mounted('imu', 'imu', config['sensors']['imu'])
    ET.SubElement(imu, 'noise', angle='0.001', angular_velocity='0.001', linear_acceleration='0.02')
    gyro = mounted('gyro', 'imu', config['sensors']['gyro'])
    ET.SubElement(gyro, 'noise', angular_velocity='0.0002')
    pressure = mounted('pressure', 'pressure', config['sensors']['pressure'])
    ET.SubElement(pressure, 'noise', pressure='5.0')
    dvl = mounted('dvl', 'dvl', config['sensors']['dvl'])
    ET.SubElement(dvl, 'specs', beam_angle='30')
    ET.SubElement(dvl, 'range', velocity='3 3 3', altitude_min='0.1', altitude_max='10')
    ET.SubElement(dvl, 'noise', velocity='0.002', altitude='0.005')
    dvl.find('ros_publisher').set('altitude_topic', '/simulation/crush/dvl/altitude')
    sonar = config['sensors']['sonar']
    scanner = mounted('sonar', 'multibeam', sonar)
    ET.SubElement(scanner, 'specs', fov='360', steps='400')  # one ray per Ping360 gradian
    ET.SubElement(scanner, 'range', distance_min=format(sonar['range'][0], 'g'), distance_max=format(sonar['range'][1], 'g'))
    if cameras:
        for name, mount in config['cameras'].items():
            camera = mounted(f'camera/{name}', 'camera', mount)
            width, height = mount['resolution']
            ET.SubElement(
                camera, 'specs', resolution_x=str(width), resolution_y=str(height), horizontal_fov=str(mount['fov'])
            )
    ET.SubElement(robot, 'ros_subscriber', thrusters='/simulation/crush/thruster_setpoints')
    ET.SubElement(robot, 'ros_publisher', thrusters='/simulation/crush/thruster_state')
    ET.SubElement(robot, 'ros_base_link_transform', publish='true')
    ET.SubElement(robot, 'world_transform', xyz=_vector(spawn), rpy='0 0 0')
    ET.indent(root, space='  ')
    return ET.ElementTree(root)


def add_robot_materials(root):
    materials = root.find('materials')
    ET.SubElement(materials, 'material', name='robot_aluminium', density='2700', restitution='0.1')
    ET.SubElement(materials, 'material', name='robot_neutral', density='1000', restitution='0')
    ET.SubElement(
        materials.find('friction_table'),
        'friction',
        material1='robot_aluminium',
        material2='pool_concrete',
        static='0.6',
        dynamic='0.4',
    )
    looks = root.find('looks')
    ET.SubElement(looks, 'look', name='robot_hull', rgb='0.85 0.65 0.18', roughness='0.4', metalness='0.5')
    ET.SubElement(looks, 'look', name='robot_propeller', rgb='0.12 0.16 0.20', roughness='0.5', metalness='0.3')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    build_robot(load_robot_config(args.config), cameras=not args.headless).write(
        args.output, encoding='utf-8', xml_declaration=True
    )
    print(f'Wrote {args.output}')


if __name__ == '__main__':
    main()
