import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Part 2 with simulated pose TF, RViz, and RQT tools."""
    puzzlebot_sim_dir = get_package_share_directory('puzzlebot_sim')

    urdf_file = os.path.join(puzzlebot_sim_dir, 'urdf', 'puzzlebot.urdf')
    rviz_config = os.path.join(
        puzzlebot_sim_dir,
        'rviz',
        'puzzlebot_rviz.rviz',
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': False},
            ],
        ),
        Node(
            package='puzzlebot_sim',
            executable='puzzlebot_sim',
            name='puzzlebot_sim',
            output='screen',
        ),
        Node(
            package='puzzlebot_sim',
            executable='puzzlebot_transforms',
            name='puzzlebot_transforms',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[
                {'use_sim_time': False},
            ],
        ),
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph',
            output='screen',
        ),
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot',
            output='screen',
        ),
    ])
