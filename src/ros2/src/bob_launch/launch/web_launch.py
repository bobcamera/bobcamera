from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bob_camera',
            executable='web_camera_video_node',
            name='web_camera_video_node',
            output='screen',
            parameters=[{'image_publish_topic': 'bob/camera/all_sky/bayer'}
            , {'image_info_publish_topic': 'bob/camera/all_sky/image_info'}
            , {'camera_info_publish_topic': 'bob/camera/all_sky/camera_info'}
            , {'is_video': False}
            , {'camera_id': 0}]
        ),
        # Node(
        #     package='bob_image_processing',
        #     executable='frame_provider_node',
        #     name='frame_provider_node',
        #     output='screen'
        # ),
        Node(
            package='bob_image_processing',
            executable='background_subtractor_node',
            name='background_subtractor_node',
            output='screen'
        ),
        # Node(
        #     package='bob_tracking',
        #     executable='track_provider_node',
        #     name='track_provider_node',
        #     output='screen'
        # ),
        # Node(
        #     package='bob_visualizers',
        #     executable='annotated_frame_provider_node',
        #     name='annotated_frame_provider_node',
        #     output='screen'
        # ),
        # Node(
        #     package='bob_visualizers',
        #     executable='frame_viewer_node',
        #     name='frame_viewer_node',
        #     output='screen',
        #     parameters=[{"topics": ["bob/camera/all_sky/bayer", "bob/frames/all_sky/foreground_mask"]}]
        # ),
        Node(
            package='bob_visualizers',
            executable='frame_bbox_viewer_node',
            name='frame_bbox_viewer_node',
            output='screen',
            parameters=[{"topics": ["bob/camera/all_sky/bayer", "bob/frames/all_sky/foreground_mask"]}]
        ),
    ])
