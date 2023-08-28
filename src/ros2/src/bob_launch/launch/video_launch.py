import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # video_file1 = os.path.join(get_package_share_directory('bob_launch'), 'videos', 'brad_drone_1.mp4')
    video_file1 = '/workspaces/bobcamera/test/fisheye_videos/brad_drone_1.mp4'
    video_file2 = '/workspaces/bobcamera/test/fisheye_videos/Dahua-20220901-184734.mp4'
    return LaunchDescription([
        Node(
            package='bob_camera',
            executable='web_camera_video_node',
            name='web_camera_video_node',
            output='screen',
            parameters=[{'image_publish_topic': 'bob/camera/all_sky/bayer'}
                        , {'image_info_publish_topic': 'bob/camera/all_sky/image_info'}
                        , {'camera_info_publish_topic': 'bob/camera/all_sky/camera_info'}
                        , {'is_video': True}
                        , {'videos': [video_file1, video_file2]}]
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
        #     parameters=[{'enable_profiling': False}, {"topics": ["bob/camera/all_sky/bayer", "bob/frames/all_sky/masked", "bob/frames/all_sky/foreground_mask"]}]
        # ),
        Node(
            package='bob_visualizers',
            executable='frame_bbox_viewer_node',
            name='frame_bbox_viewer_node',
            output='screen',
            parameters=[{"topics": ["bob/camera/all_sky/bayer", "bob/frames/all_sky/foreground_mask"]}]
        ),
    ])
