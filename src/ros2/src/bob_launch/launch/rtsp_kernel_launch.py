import os
import launch
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the config directory
    config_dir = os.path.join(get_package_share_directory('bob_launch'), 'config')
    param_config = os.path.join(config_dir, "ipcamera.yaml")
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)["ipcamera"]["ros__parameters"]

    # Alternatively can use "package://" as discussed:
    # https://answers.ros.org/question/333521/ros2-url-to-camera_info-yaml-not-being-recognized/
    config_file = 'file://' + os.path.join(config_dir, "camera_info.yaml")

    """Generate launch description with multiple components."""
    rstp_container = ComposableNodeContainer(
            name='rstp_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ros2_ipcamera',
                    plugin='ros2_ipcamera::IpCamera',
                    name='ipcamera',
                    remappings=[
                        ('/ipcamera/image_raw', 'bob/camera/all_sky/bayer'),
                        ('/ipcamera/camera_info', 'bob/camera/all_sky/camera_info')],
                    parameters=[
                        params,
                        {"camera_calibration_file": config_file}],
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ],
            output='screen',
    )

    resize_container = ComposableNodeContainer(
            name='track_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
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

    return launch.LaunchDescription([rstp_container, resize_container])