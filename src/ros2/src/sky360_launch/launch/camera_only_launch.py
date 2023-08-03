from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sky360_camera',
            executable='all_sky_publisher_node',
            name='all_sky_publisher_node',
            output='screen',
            parameters=[{'enable_profiling': False}, {'exposure': 20000}, {'gain': 30}, {'auto_exposure': True}]
        ),
        Node(
            package='sky360_visualizers',
            executable='frame_viewer_node',
            name='frame_viewer_node',
            output='screen',
            parameters=[{'enable_profiling': False}, {"topics": ["sky360/camera/all_sky/bayer"]}]
        ),
    ])
