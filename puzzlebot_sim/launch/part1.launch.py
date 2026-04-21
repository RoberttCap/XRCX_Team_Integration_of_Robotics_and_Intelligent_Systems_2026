from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Part 1 test launch without RViz visualization.

    Test simulations:
    - Input commands /cmd_vel:
      - test linear speed and position
      - test angular speed and position
    - Use teleop_twist_keyboard to test results
    - Use rqt_graph to verify node/topic connections
    - Use rqt_plot to plot position and wheel speed and verify results
    """
    return LaunchDescription([
        Node(
            package='puzzlebot_sim',
            executable='puzzlebot_sim',
            name='puzzlebot_sim',
            output='screen',
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
