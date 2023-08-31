import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    launch_package_dir = get_package_share_directory('bob_launch')

    vizualise_container = ComposableNodeContainer(
        name='display_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='bob_visualizers',
                plugin='FrameViewer',
                name='frame_viewer_node',
                parameters=[{"topics": ["bob/camera/all_sky/bayer/resized", "bob/frames/all_sky/foreground_mask/resized", "bob/frames/annotated/resized"]}],
                extra_arguments=[{'use_intra_process_comms': True}])                                                            
            ],
            output='screen',
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/kernel_launch.py'])
        ),

        vizualise_container
    ])