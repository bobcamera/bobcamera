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
        name='track_container',
        namespace=namespace,
        arguments=['--ros-args', '--log-level', loglevel],
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[

            ComposableNode(
               package='bob_camera',
               plugin='WebCameraVideo',
               name='rtsp_camera_node',
               namespace=namespace,
               parameters = [config],
               extra_arguments=[{'use_intra_process_comms': True}],
               condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp'" ]))
            ),
            ComposableNode(
                package='bob_camera',
                plugin='WebCameraVideo',
                name='rtsp_overlay_camera_node',
                namespace=namespace,
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp_overlay'" ]))
            ),
            ComposableNode(
                package='bob_camera',
                plugin='WebCameraVideo',
                name='web_camera_video_node',
                namespace=namespace,
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'video'" ])),
            ),
            ComposableNode(
                package='bob_camera',
                plugin='WebCameraVideo',
                name='usb_camera_node',
                namespace=namespace,
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'usb'" ])),
            ),
            ComposableNode(
                package='bob_simulator', 
                plugin='MovingObjectsSimulation', 
                name='simulated_frame_provider_node',
                namespace=namespace,
                parameters = [config],
                remappings=[
                    ('bob/simulation/output_frame', 'bob/frames/allsky/original')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'simulate'" ])),  
            ) , 
            ComposableNode(
                package='bob_camera',
                plugin='WebCameraVideo',
                name='web_camera_video_overlay_node',
                namespace=namespace,
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'video_overlay'" ])),
            ),
            ComposableNode(
                package='bob_simulator',
                plugin='SimulationOverlayProviderNode',  
                name='simulation_overlay_provider_node',
                namespace=namespace,
                parameters = [config],
                remappings=[('bob/simulation/output_frame', 'bob/frames/allsky/original')],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([
                    LaunchConfiguration('source_arg'), 
                    " == 'rtsp_overlay'", 
                    " or ", 
                    LaunchConfiguration('source_arg'), 
                    " == 'video_overlay'"]))
            ),
            ComposableNode(
                package='bob_image_processing',  
                plugin='MaskApplication',  
                name='detection_mask_application_node',
                namespace=namespace,
                parameters = [config],
                remappings=[
                    ('bob/mask/override', 'bob/mask/detection/override'),
                    ('bob/mask/source', 'bob/frames/allsky/original'),
                    ('bob/mask/target', 'bob/frames/allsky/masked/detection')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='bob_image_processing',  
                plugin='MaskApplication',  
                name='privacy_mask_application_node',
                namespace=namespace,
                parameters = [config],
                remappings=[
                    ('bob/mask/override', 'bob/mask/privacy/override'),
                    ('bob/mask/source', 'bob/frames/allsky/original'),
                    ('bob/mask/target', 'bob/frames/allsky/masked/privacy')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),            
            #The one Background Subtractor Node to rule them all:
            ComposableNode(
                package='bob_image_processing',
                plugin='BackgroundSubtractor2',
                name='background_subtractor_v2_node',
                namespace=namespace,
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}]
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
            # This is used for the star mask so needs to be included here
            ComposableNode(
                package='bob_image_processing',
                plugin='FrameResizer',
                name='original_frame_resizer_node',
                namespace=namespace,
                remappings=[
                    ('bob/resizer/source', 'bob/frames/allsky/original'),
                    ('bob/resizer/target', 'bob/frames/allsky/original/resized')],
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
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