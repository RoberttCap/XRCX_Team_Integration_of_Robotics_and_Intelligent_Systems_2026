import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the closed-loop Puzzlebot simulation stack."""
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
            executable='localisation',
            name='localisation',
            output='screen',
        ),
        Node(
            package='puzzlebot_sim',
            executable='joint_states',
            name='joint_states',
            output='screen',
        ),
        Node(
            package='puzzlebot_sim',
            executable='control',
            name='control',
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
            package='rqt_tf_tree',
            executable='rqt_tf_tree',
            name='rqt_tf_tree',
            output='screen',
        ),
    ])
