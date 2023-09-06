import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    video_file1 = '/workspaces/bobcamera/test/fisheye_videos/brad_drone_1.mp4'
    video_file2 = '/workspaces/bobcamera/test/fisheye_videos/Dahua-20220901-184734.mp4'

    """Generate launch description with multiple components."""
    kernel_container = ComposableNodeContainer(
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
                ComposableNode(
                    package='bob_image_processing',
                    plugin='BackgroundSubtractor',
                    name='background_subtractor_node',
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='bob_tracking',
                    plugin='TrackProvider',
                    name='track_provider_node',
                    extra_arguments=[{'use_intra_process_comms': True}]),
                #ComposableNode(
                #    package='bob_recorder',
                #    plugin='RosbagRecorder',
                #    name='rosbag_recorder_node',
                #    extra_arguments=[{'use_intra_process_comms': True}]),                    
                ComposableNode(
                    package='bob_image_processing',
                    plugin='AnnotatedFrameProvider',
                    name='annotated_frame_provider_node',
                    extra_arguments=[{'use_intra_process_comms': True}]),
                
                # Nodes for resizing the image in order to stick it on the network for display
                ComposableNode(
                    package='bob_image_processing',
                    plugin='FrameResizer',
                    name='bayer_frame_resizer_node',
                    remappings=[
                        ('bob/resizer/source', 'bob/camera/all_sky/bayer'),
                        ('bob/resizer/target', 'bob/camera/all_sky/bayer/resized')],
                    parameters=[{'resize_height': 960}],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='bob_image_processing',
                    plugin='FrameResizer',
                    name='foreground_mask_frame_resizer_node',
                    remappings=[
                        ('bob/resizer/source', 'bob/frames/all_sky/foreground_mask'),
                        ('bob/resizer/target', 'bob/frames/all_sky/foreground_mask/resized')],
                    parameters=[{'resize_height': 960}],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='bob_image_processing',
                    plugin='FrameResizer',
                    name='annotated_frame_resizer_node',
                    remappings=[
                        ('bob/resizer/source', 'bob/frames/annotated'),
                        ('bob/resizer/target', 'bob/frames/annotated/resized')],
                    parameters=[{'resize_height': 960}],
                    extra_arguments=[{'use_intra_process_comms': True}]),                    
            ],
            output='screen',
    )

    return launch.LaunchDescription([kernel_container])