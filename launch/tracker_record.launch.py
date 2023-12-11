"""
Launch file to run tracker with a dummy publisher and record topics into a rosbag.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch

def generate_launch_description():
    return LaunchDescription([

        launch.actions.ExecuteProcess(
            cmd=['ros2','bag','record',
                 '/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d_gt/center',  
                 '/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d/center',
                 '/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d/marker_point0',
                 '/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d/marker_point1',
                 '/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d/marker_point2',
                 '/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d/marker_point3',
                 '/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d/marker_point4',
                 '/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d/marker_point5',
                 '/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d/marker_point7',
                 '/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d/marker_point8',
                 '/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d/marker_point9',
                #  '/tractor/radar/dynamic/obstacle_detection/tracking/marker_poly0',
                #  '/tractor/radar/dynamic/obstacle_detection/tracking/marker_poly1',
                #  '/tractor/radar/dynamic/obstacle_detection/tracking/marker_poly2',
                #  '/tractor/radar/dynamic/obstacle_detection/tracking/marker_poly3',
                #  '/tractor/radar/dynamic/obstacle_detection/tracking/marker_poly4',
                #  '/tractor/radar/dynamic/obstacle_detection/tracking/marker_poly5',
                #  '/tractor/radar/dynamic/obstacle_detection/tracking/marker_poly6',
                #  '/tractor/radar/dynamic/obstacle_detection/tracking/marker_poly7',
                #  '/tractor/radar/dynamic/obstacle_detection/tracking/marker_poly8',
                #  '/tractor/radar/dynamic/obstacle_detection/tracking/marker_poly9', 
                 '/tractor/radar/dynamic/obstacle_detection/tracking/marker_point0',
                 '/tractor/radar/dynamic/obstacle_detection/tracking/marker_point1',
                 '/tractor/radar/dynamic/obstacle_detection/tracking/marker_point2',
                 '/tractor/radar/dynamic/obstacle_detection/tracking/marker_point3',
                 '/tractor/radar/dynamic/obstacle_detection/tracking/marker_point4',
                 '/tractor/radar/dynamic/obstacle_detection/tracking/marker_point5',
                 '/tractor/radar/dynamic/obstacle_detection/tracking/marker_point6',
                 '/tractor/radar/dynamic/obstacle_detection/tracking/marker_point7',
                 '/tractor/radar/dynamic/obstacle_detection/tracking/marker_point8',
                 '/tractor/radar/dynamic/obstacle_detection/tracking/marker_point9', 
                 ],
            output='screen'
        ),

        Node(
            package='bb_tracker',
            executable='sort_tracker',
            # name='bb_2d_tracker',
            # parameters=[{"use_sim_time": True}],
        ),

        # Node(
        #     package='bb_tracker',
        #     executable='dummy_pub',
        #     # name='dummy_pub',
        #     # parameters=[{"use_sim_time": True}],
        # ),

        # # Node to publish dummy clustering data as polygons for RVIZ
        # Node(
        #     package='pointcloud_toolbox',
        #     executable='publish_polygon_array_marker_node',
        #     name='polygon_array_marker_pub',
        #     namespace="/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d",
        #     remappings=[
        #         ('/input', "/tractor/radar/dynamic/obstacle_detection/clustering/bb_2d"),
        #     ],
        # ),
        
    ])
