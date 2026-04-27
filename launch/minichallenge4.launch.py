import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch import LaunchDescription
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node


def robot_group(namespace, robot_desc, x0, y0, theta0):
    """Create one namespaced Puzzlebot stack."""
    return GroupAction([
        PushRosNamespace(namespace),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'frame_prefix': namespace + '/'},
                {'use_sim_time': False},
            ],
            remappings=[
                ('/joint_states', 'joint_states'),
            ],
        ),
        Node(
            package='puzzlebot_sim',
            executable='puzzlebot_sim',
            name='puzzlebot_sim',
            output='screen',
            parameters=[
                {'x0': x0},
                {'y0': y0},
                {'theta0': theta0},
                {'odom_frame': namespace + '/odom'},
            ],
        ),
        Node(
            package='puzzlebot_sim',
            executable='localisation',
            name='localisation',
            output='screen',
            parameters=[
                {'x0': x0},
                {'y0': y0},
                {'theta0': theta0},
                {'odom_frame': namespace + '/odom'},
                {'base_frame': namespace + '/base_footprint'},
            ],
        ),
        Node(
            package='puzzlebot_sim',
            executable='joint_states',
            name='joint_states',
            output='screen',
            parameters=[
                {'odom_frame': namespace + '/odom'},
                {'base_frame': namespace + '/base_footprint'},
            ],
        ),
        Node(
            package='puzzlebot_sim',
            executable='control',
            name='control',
            output='screen',
        ),
    ])


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
        robot_group('robot1', robot_desc, 1.0, 1.0, 0.0),
        robot_group('robot2', robot_desc, 1.0, 0.5, 0.0),
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
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph',
            output='screen',
        ),
    ])
