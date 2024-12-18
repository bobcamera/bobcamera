import os
import launch
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PythonExpression, LaunchConfiguration, EnvironmentVariable 
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the application config
    #config = os.path.join(get_package_share_directory('bob_launch'), 'config', 'app_config.yaml')
    config = 'assets/config/app_config.yaml'
    namespace = EnvironmentVariable('BOB_NAMESPACE', default_value="")
    loglevel = EnvironmentVariable('BOB_LOGLEVEL', default_value="INFO")

    kernel_container = ComposableNodeContainer(
        name='kernel_container',
        namespace=namespace,
        arguments=['--ros-args', '--log-level', loglevel],
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[

            ComposableNode(
                package='bob_camera',
                plugin='CameraBGS',
                name='rtsp_camera_node',
                namespace=namespace,
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp'" ]))
            ),
            ComposableNode(
                package='bob_camera',
                plugin='CameraBGS',
                name='rtsp_overlay_camera_node',
                namespace=namespace,
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp_overlay'" ]))
            ),
            ComposableNode(
                package='bob_camera',
                plugin='CameraBGS',
                name='web_camera_video_node',
                namespace=namespace,
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'video'" ])),
            ),
            ComposableNode(
                package='bob_camera',
                plugin='CameraBGS',
                name='usb_camera_node',
                namespace=namespace,
                parameters = [config],
                remappings=[('bob/mask/override', 'bob/mask/privacy/override')],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'usb'" ])),
            ),
            ComposableNode(
                package='bob_camera',
                plugin='CameraBGS',
                name='web_camera_video_overlay_node',
                namespace=namespace,
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True, 'RCUTILS_LOGGING_CONFIG_FILE': '/workspaces/bobcamera/src/ros2/config/ros2_log_config.yaml'}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'video_overlay'" ])),
            ),
            ComposableNode(
                package='bob_tracking',
                plugin='TrackProvider',
                name='track_provider_node',
                namespace=namespace,
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='bob_recorder',
                plugin='RecordManager',
                name='allsky_recorder_node',
                namespace=namespace,
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('enable_recording_arg'), " == True"])),  
            ),
            ComposableNode(
                package='bob_image_processing',
                plugin='AnnotatedFrameProvider',
                name='annotated_frame_provider_node',
                namespace=namespace,
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Compressing Nodes
            ComposableNode(
                package='bob_image_processing',
                plugin='FrameCompressor',
                name='annotated_frame_compressor_node',
                namespace=namespace,                
                remappings=[
                    ('bob/compressor/source', 'bob/frames/annotated/resized'),
                    ('bob/compressor/target', 'bob/frames/annotated/resized/compressed')],
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='bob_image_processing',
                plugin='FrameCompressor',
                name='detection_masked_compressor_node',
                namespace=namespace,
                remappings=[
                    ('bob/compressor/source', 'bob/frames/allsky/original/resized'),
                    ('bob/compressor/target', 'bob/frames/allsky/original/resized/compressed')],
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}]),
           ComposableNode(
                package='bob_image_processing',
                plugin='FrameCompressor',
                name='foreground_mask_compressor_node',
                namespace=namespace,
                remappings=[
                    ('bob/compressor/source', 'bob/frames/foreground_mask/resized'),
                    ('bob/compressor/target', 'bob/frames/foreground_mask/resized/compressed')],
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}]),                
        ],
        output='screen',
    )    

    return launch.LaunchDescription([

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp_overlay'" ])),
            msg=['Frame source is set to: RTSP OVERLAY.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'simulate'" ])),
            msg=['Frame source is set to: SIMULATION.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'video_overlay'" ])),
            msg=['Frame source is set to: VIDEO OVERLAY.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'video'" ])),
            msg=['Frame source is set to: VIDEO.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp'" ])),
            msg=['Frame source is set to: RTSP Camera.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'usb'" ])),
            msg=['Frame source is set to: USB Camera.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('operation_arg'), " == 'standard'" ])),
            msg=['Mode of operation: STANDARD.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('operation_arg'), " == 'headless'" ])),
            msg=['Mode of operation: HEADLESS.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_arg'), " == 'minimal'" ])),
            msg=['Tracking sensitivity is set to: MINIMAL.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_arg'), " == 'low'" ])),
            msg=['Tracking sensitivity is set to: LOW.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_arg'), " == 'medium'" ])),
            msg=['Tracking sensitivity is set to: MEDIUM.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_arg'), " == 'high'" ])),
            msg=['Tracking sensitivity is set to: HIGH.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('enable_recording_arg'), " == True" ])),
            msg=['Recording is set to: ON.']),

        kernel_container
        ]
    )