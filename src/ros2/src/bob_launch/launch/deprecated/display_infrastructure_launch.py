import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import LogInfo
from launch.substitutions import PythonExpression, LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the application config
    #config = os.path.join(get_package_share_directory('bob_launch'), 'config', 'app_config.yaml')
    config = 'assets/config/app_config.yaml'
    namespace = EnvironmentVariable('BOB_NAMESPACE', default_value="")
    loglevel = EnvironmentVariable('BOB_LOGLEVEL', default_value="INFO")

    display_container = ComposableNodeContainer(
        name='display_container',
        namespace=namespace,
        arguments=['--ros-args', '--log-level', loglevel],
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='bob_visualizers',
                plugin='FrameViewer',
                name='multi_frame_viewer_node',
                namespace=namespace,
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([
                    LaunchConfiguration('enable_visualiser_arg'), 
                    " == True"])))
            ],
            output='screen',
    )

    # compress_container = ComposableNodeContainer(
    #     name='compress_container',
    #     namespace=namespace,
    #     arguments=['--ros-args', '--log-level', loglevel],
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[

    #         # Nodes for compression the image (jpg) for display on the web
    #         ComposableNode(
    #             package='bob_image_processing',
    #             plugin='FrameCompressor',
    #             name='annotated_frame_compressor_node',
    #             namespace=namespace,                
    #             remappings=[
    #                 ('bob/compressor/source', 'bob/frames/annotated/resized'),
    #                 ('bob/compressor/target', 'bob/frames/annotated/resized/compressed')],
    #             parameters = [config],
    #             extra_arguments=[{'use_intra_process_comms': True}]),
    #         ComposableNode(
    #             package='bob_image_processing',
    #             plugin='FrameCompressor',
    #             name='detection_masked_compressor_node',
    #             namespace=namespace,
    #             remappings=[
    #                 ('bob/compressor/source', 'bob/frames/allsky/masked/detection/resized'),
    #                 ('bob/compressor/target', 'bob/frames/allsky/masked/detection/resized/compressed')],
    #             parameters = [config],
    #             extra_arguments=[{'use_intra_process_comms': True}]),
    #         ComposableNode(
    #             package='bob_image_processing',
    #             plugin='FrameCompressor',
    #             name='privacy_masked_compressor_node',
    #             namespace=namespace,
    #             remappings=[
    #                 ('bob/compressor/source', 'bob/frames/allsky/masked/privacy/resized'),
    #                 ('bob/compressor/target', 'bob/frames/allsky/masked/privacy/resized/compressed')],
    #             parameters = [config],
    #             extra_arguments=[{'use_intra_process_comms': True}]),                
    #        ComposableNode(
    #             package='bob_image_processing',
    #             plugin='FrameCompressor',
    #             name='foreground_mask_compressor_node',
    #             namespace=namespace,
    #             remappings=[
    #                 ('bob/compressor/source', 'bob/frames/foreground_mask/resized'),
    #                 ('bob/compressor/target', 'bob/frames/foreground_mask/resized/compressed')],
    #             parameters = [config],
    #             extra_arguments=[{'use_intra_process_comms': True}]),                
    #     ],
    #     output='screen',
    # )

    return LaunchDescription([

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('enable_visualiser_arg'), " == False" ])),
            msg=['Visualiser disabled.']),

        display_container, 
        # compress_container
        ]
    )
