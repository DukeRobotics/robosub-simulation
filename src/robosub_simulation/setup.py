from glob import glob

from setuptools import find_packages, setup

package_name = 'robosub_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scenarios', glob('scenarios/*.scn')),
        ('share/' + package_name + '/scenarios', glob('scenarios/*.xml')),
        ('share/' + package_name + '/meshes', glob('meshes/*.obj')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Duke Robotics Club',
    maintainer_email='hello@duke-robotics.com',
    description='Stonefish pool simulation for Duke RoboSub.',
    license='GPL-3.0-only',
    entry_points={
        'console_scripts': [
            'generate_pool = robosub_simulation.pool:main',
            'generate_robot = robosub_simulation.robot:main',
            'teleop_mixer = robosub_simulation.teleop:main',
        ]
    },
)
