import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cell_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tutorial',
    maintainer_email='tutorial@example.invalid',
    description='Digital twin of the palletizing cell',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'world_model = cell_sim.world_model:main',
        'mock_gripper = cell_sim.mock_gripper:main',
        'spawn_scene = cell_sim.spawn_scene:main',
        'mock_camera = cell_sim.mock_camera:main',
        'mock_conveyor = cell_sim.mock_conveyor:main',
    ]},
)
