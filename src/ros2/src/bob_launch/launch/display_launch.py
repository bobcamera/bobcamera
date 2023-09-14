from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import LogInfo
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch.conditions import IfCondition

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
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([
                    LaunchConfiguration('enable_visualiser_arg'), 
                    " == True", 
                    " and ", 
                    LaunchConfiguration('optimised_arg'), 
                    " == False"]))),
            ComposableNode(
                package='bob_visualizers',
                plugin='FrameViewer',
                name='frame_viewer_node',
                parameters=[{"topics": ["bob/frames/annotated/resized"]}],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([
                    LaunchConfiguration('enable_visualiser_arg'), 
                    " == True",
                    " and ", 
                    LaunchConfiguration('optimised_arg'), 
                    " == True"]))),
            ],
            output='screen',
    )

    compress_container = ComposableNodeContainer(
        name='compress_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[

            # Nodes for compression the image (jpg) for display on the web
            ComposableNode(
                package='bob_image_processing',
                plugin='FrameCompressor',
                name='annotated_frame_compressor_node',
                remappings=[
                    ('bob/compressor/source', 'bob/frames/annotated/resized'),
                    ('bob/compressor/target', 'bob/frames/annotated/resized/compressed')],
                parameters=[{'compression_quality': 95}],
                extra_arguments=[{'use_intra_process_comms': True}]),

            # New node for compressing the bayer image
            ComposableNode(
                package='bob_image_processing',
                plugin='FrameCompressor',
                name='bayer_compressor_node',
                remappings=[
                    ('bob/compressor/source', 'bob/camera/all_sky/bayer/resized'),
                    ('bob/compressor/target', 'bob/camera/all_sky/bayer/resized/compressed')],
                parameters=[{'compression_quality': 95}],
                extra_arguments=[{'use_intra_process_comms': True}]),             
        ],
        output='screen',
    )

    return LaunchDescription([

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('optimised_arg'), " == True" ])),
            msg=['Optimisation enabled.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('enable_visualiser_arg'), " == False" ])),
            msg=['Visualiser disabled.']),

        display_container, 
        compress_container
        ]
    )
