from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    display_container = ComposableNodeContainer(
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

    return LaunchDescription([display_container])