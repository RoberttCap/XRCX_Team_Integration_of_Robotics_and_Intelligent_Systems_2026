import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # rqt_plot - to visualize position and velocities
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot',
            arguments=[
                '/pos_sim/pose/position/x',
                '/pos_sim/pose/position/y',
                '/wheel_velocities/data[0]',
                '/wheel_velocities/data[1]'
            ]
        ),

        # rqt_graph - to see the node and topic connections
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph'
        ),
    ])
