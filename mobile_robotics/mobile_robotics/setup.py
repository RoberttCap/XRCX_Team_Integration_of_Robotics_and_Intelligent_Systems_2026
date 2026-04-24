from setuptools import find_packages, setup

package_name = 'mobile_robotics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karinam',
    maintainer_email='karifer16.26@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'test_node = mobile_robotics.test_node:main',
            'laser_scan_subscriber = mobile_robotics.laser_scan_subscriber:main',
            'closest_object_follower = mobile_robotics.closest_object_follower:main',
            'obstacle_avoidance = mobile_robotics.obstacle_avoidance:main',      
        ],
    },
)
