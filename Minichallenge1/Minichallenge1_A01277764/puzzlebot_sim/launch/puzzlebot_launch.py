import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('puzzlebot_sim')

    urdf_file = os.path.join(pkg, 'urdf', 'urdf.urdf')
    rviz_file = os.path.join(pkg, 'rviz', 'rviz.rviz')

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

        # rqt_graph
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph'
        ),
    ])