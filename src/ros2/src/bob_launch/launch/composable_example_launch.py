import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    video_file1 = '/workspaces/bobcamera/test/fisheye_videos/brad_drone_1.mp4'
    video_file2 = '/workspaces/bobcamera/test/fisheye_videos/Dahua-20220901-184734.mp4'

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='track_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='bob_camera',
                    plugin='WebCameraVideo',
                    name='web_camera_video_node',
                    parameters=[{'image_publish_topic': 'bob/camera/all_sky/bayer'}
                        , {'image_info_publish_topic': 'bob/camera/all_sky/image_info'}
                        , {'camera_info_publish_topic': 'bob/camera/all_sky/camera_info'}
                        , {'is_video': True}
                        , {'videos': [video_file1, video_file2]}
                        , {'resize_height': 0}],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='bob_image_processing',
                #     plugin='bob_image_processing::background_subtractor_node',
                #     name='background_subtractor_node',
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='bob_tracking',
                #     plugin='bob_tracking::track_provider_node',
                #     name='track_provider_node',
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='bob_image_processing',
                #     plugin='bob_image_processing::annotated_frame_provider_node',
                #     name='annotated_frame_provider_node',
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='bob_visualizers',
                    plugin='FrameViewer',
                    name='frame_viewer_node',
                    parameters=[{"topics": ["bob/camera/all_sky/bayer", "bob/frames/annotated", "bob/frames/all_sky/foreground_mask"]}],
                    extra_arguments=[{'use_intra_process_comms': True}])                                                            
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])