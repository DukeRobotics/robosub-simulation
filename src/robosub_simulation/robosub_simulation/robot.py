"""Build the provisional Crush Stonefish robot from CAD-derived properties."""

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from robosub_simulation.pool import _vector


def load_robot_config(path):
    path = Path(path)
    config = yaml.safe_load(path.read_text())
    config['model_data'] = yaml.safe_load((path.parent / config['model']).read_text())
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
        position = [a + b for a, b in zip(model['center_of_mass'], definition['offset'], strict=True)]
        ET.SubElement(thruster, 'origin', xyz=_vector(position), rpy=_vector(definition['rpy']))
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

    def sensor(name, kind, rate, offset=(0, 0, 0), rpy=(0, 0, 0)):
        element = ET.SubElement(robot, 'sensor', name=name, type=kind, rate=str(rate))
        ET.SubElement(element, 'link', name='base_link')
        position = [a + b for a, b in zip(model['center_of_mass'], offset, strict=True)]
        ET.SubElement(element, 'origin', xyz=_vector(position), rpy=_vector(rpy))
        ET.SubElement(element, 'ros_publisher', topic=f'/simulation/crush/{name}')
        ET.SubElement(element, 'ros_service', set_enabled=f'/simulation/crush/{name}/set_enabled')
        return element

    sensor('ground_truth', 'odometry', 30)
    imu = sensor('imu', 'imu', 50)
    ET.SubElement(imu, 'noise', angle='0.001', angular_velocity='0.001', linear_acceleration='0.02')
    pressure = sensor('pressure', 'pressure', 20)
    ET.SubElement(pressure, 'noise', pressure='5.0')
    dvl = sensor('dvl', 'dvl', 20, offset=(0, 0, 0.16), rpy=(math.pi, 0, 0))
    ET.SubElement(dvl, 'specs', beam_angle='30')
    ET.SubElement(dvl, 'range', velocity='3 3 3', altitude_min='0.1', altitude_max='10')
    ET.SubElement(dvl, 'noise', velocity='0.002', altitude='0.005')
    dvl.find('ros_publisher').set('altitude_topic', '/simulation/crush/dvl/altitude')
    if cameras:
        camera = sensor('camera/front', 'camera', 5, offset=(0.36, 0, 0), rpy=(math.pi / 2, 0, math.pi / 2))
        ET.SubElement(camera, 'specs', resolution_x='320', resolution_y='240', horizontal_fov='80')
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
