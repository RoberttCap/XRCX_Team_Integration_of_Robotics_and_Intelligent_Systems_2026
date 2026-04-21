import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('puzzlebot_sim')

    urdf_file = os.path.join(pkg, 'urdf', 'puzzlebot.urdf')
    rviz_file = os.path.join(pkg, 'rviz', 'puzzlebot_rviz.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }]
        ),

        
        Node(
            package='puzzlebot_sim',
            executable='joint_state_publisher',
            name='puzzlebot_publisher',
            parameters=[{'use_sim_time': False}]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': False}]
        ),

        # Controller Node
        Node(
            package='puzzlebot_sim',
            executable='controller',
            name='controller',
            parameters=[{'use_sim_time': False}]
        ),

        # Differential Drive Node
        Node(
            package='puzzlebot_sim',
            executable='differential_drive',
            name='differential_drive',
            parameters=[{'use_sim_time': False}]
        ),

        # Localization Node
        Node(
            package='puzzlebot_sim',
            executable='localization',
            name='localization',
            parameters=[{'use_sim_time': False}]
        ),
    ])