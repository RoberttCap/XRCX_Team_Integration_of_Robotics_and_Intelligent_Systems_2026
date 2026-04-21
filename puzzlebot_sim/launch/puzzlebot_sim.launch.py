import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the complete stack for Part 1, 2 and 3."""
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
        # URDF model for RViz.
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
        # Part 1: simulator node.
        Node(
            package='puzzlebot_sim',
            executable='puzzlebot_sim',
            name='puzzlebot_sim',
            output='screen',
        ),
        # Part 2: dead-reckoning node.
        Node(
            package='puzzlebot_sim',
            executable='localisation',
            name='localisation',
            output='screen',
        ),
        # Part 3: TF + joint_states node based on /odom, /wr and /wl.
        Node(
            package='puzzlebot_sim',
            executable='joint_states',
            name='joint_states',
            output='screen',
        ),
        # Visualization tools.
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
            package='rqt_tf_tree',
            executable='rqt_tf_tree',
            name='rqt_tf_tree',
            output='screen',
        ),
    ])
