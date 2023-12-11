"""
Launch file to run tracker with a rosbag.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='bb_tracker',
            executable='sort_tracker',
            # name='bb_2d_tracker',
            # parameters=[{"use_sim_time": True}],
        ),

    ])
