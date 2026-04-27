from setuptools import find_namespace_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_namespace_packages(
        include=[package_name, package_name + '.*'],
        exclude=['test'],
    ),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.[yma]*')),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz')),
        ),
        (
            os.path.join('share', package_name, 'meshes'),
            glob(os.path.join('meshes', '*.stl')),
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karifmaldonado',
    maintainer_email='karifmaldonado26@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control = puzzlebot_sim.control:main',
            'goal_input = puzzlebot_sim.goal_input:main',
            'joint_states = puzzlebot_sim.joint_states:main',
            'localisation = puzzlebot_sim.localisation:main',
            'puzzlebot_sim = puzzlebot_sim.puzzlebot_sim:main',
            'puzzlebot_transforms = puzzlebot_sim.transforms:main',
        ],
    },
)
