from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sky360_camera',
            executable='qhy_node',
            name='all_sky_publisher_node',
            output='screen',
            parameters=[{'camera_id': ''}
                        , {'image_publish_topic': 'sky360/camera/all_sky/bayer'}
                        , {'image_info_publish_topic': 'sky360/camera/all_sky/image_info'}
                        , {'camera_info_publish_topic': 'sky360/camera/all_sky/camera_info'}
                        , {'enable_profiling': False}
                        , {'exposure': 20000}
                        , {'gain': 30}
                        , {'auto_exposure': True}]
        ),
        Node(
            package='sky360_visualizers',
            executable='frame_viewer_node',
            name='frame_viewer_node',
            output='screen',
            parameters=[{'topics': ['sky360/camera/all_sky/bayer']}]
        ),
    ])
