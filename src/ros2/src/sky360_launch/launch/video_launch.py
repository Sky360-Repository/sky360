import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    video_file = os.path.join(get_package_share_directory('sky360_launch'), 'videos', 'brad_drone_1.mp4')
    return LaunchDescription([
        Node(
            package='sky360_camera',
            executable='web_camera_publisher_node',
            name='web_camera_publisher_node',
            output='screen',
            parameters=[{'is_video': True}, {'video_path': video_file}]
        ),
        Node(
            package='sky360_image_processing',
            executable='frame_provider_node',
            name='frame_provider_node',
            output='screen'
        ),
        Node(
            package='sky360_image_processing',
            executable='background_subtractor_node',
            name='background_subtractor_node',
            output='screen'
        ),
        # Node(
        #     package='sky360_tracking',
        #     executable='track_provider_node',
        #     name='track_provider_node',
        #     output='screen'
        # ),
        # Node(
        #     package='sky360_visualizers',
        #     executable='annotated_frame_provider_node',
        #     name='annotated_frame_provider_node',
        #     output='screen'
        # ),
        # Node(
        #     package='sky360_visualizers',
        #     executable='frame_viewer_node',
        #     name='frame_viewer_node',
        #     output='screen',
        #     parameters=[{'enable_profiling': False}, {"topics": ["sky360/camera/all_sky/bayer", "sky360/frames/all_sky/masked", "sky360/frames/all_sky/foreground_mask"]}]
        # ),
        Node(
            package='sky360_visualizers',
            executable='frame_bbox_viewer_node',
            name='frame_bbox_viewer_node',
            output='screen',
            parameters=[{'enable_profiling': False}, {"topics": ["sky360/frames/all_sky/masked", "sky360/frames/all_sky/foreground_mask"]}]
        ),
    ])
