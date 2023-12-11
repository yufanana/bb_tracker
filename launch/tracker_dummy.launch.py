"""
Launch file to run tracker with a dummy publisher..
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():


    return LaunchDescription([

        Node(
            package='bb_tracker',
            executable='sort_tracker',
            # name='sort_tracker',
            # parameters=[{"use_sim_time": True}],
        ),

        Node(
            package='bb_tracker',
            executable='dummy_pub',
            # name='dummy_pub',
            # parameters=[{"use_sim_time": True}],
        ),

        # Node to publish tracking results as polygons for RVIZ
        # Node(
        #     package='pointcloud_toolbox',
        #     executable='publish_polygon_array_marker_node',
        #     name='polygon_array_marker_pub',
        #     namespace="/tractor/radar/dynamic/obstacle_detection/tracking/bb_2d",
        #     remappings=[
        #         ('/input', "/tractor/radar/dynamic/obstacle_detection/tracking/bb_2d"),
        #     ],
        # ),


        # Node to publish dummy clustering data as polygons for RVIZ
        Node(
            package='pointcloud_toolbox',
            executable='publish_polygon_array_marker_node',
            name='polygon_array_marker_pub',
            namespace="/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d",
            remappings=[
                ('/input', "/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d"),
            ],
        ),

    ])
