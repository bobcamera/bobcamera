import os
import launch
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PythonExpression, LaunchConfiguration 
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the application config
    #config = os.path.join(get_package_share_directory('bob_launch'), 'config', 'app_config.yaml')
    config = 'assets/config/app_config.yaml'

    kernel_container = ComposableNodeContainer(
        name='track_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[

            ComposableNode(
               package='bob_camera',
               plugin='WebCameraVideo',
               name='rtsp_camera_node',
               parameters = [config],
               extra_arguments=[{'use_intra_process_comms': True}],
               condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp'" ]))
            ),
            ComposableNode(
                package='bob_camera',
                plugin='WebCameraVideo',
                name='rtsp_overlay_camera_node',
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp_overlay'" ]))
            ),
            ComposableNode(
                package='bob_camera',
                plugin='WebCameraVideo',
                name='web_camera_video_node',
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'video'" ])),
            ),
            ComposableNode(
                package='bob_camera',
                plugin='WebCameraVideo',
                name='usb_camera_node',
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'usb'" ])),
            ),
            ComposableNode(
                package='bob_simulator', 
                plugin='MovingObjectsSimulation', 
                name='simulated_frame_provider_node',  
                parameters = [config],
                remappings=[
                    ('bob/simulation/output_frame', 'bob/camera/all_sky/bayer')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'simulate'" ])),  
            ) , 
            ComposableNode(
                package='bob_camera',
                plugin='WebCameraVideo',
                name='web_camera_video_overlay_node',
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'video_overlay'" ])),
            ),
            ComposableNode(
                package='bob_simulator',
                plugin='SimulationOverlayProviderNode',  
                name='simulation_overlay_provider_node',
                parameters = [config],
                remappings=[('bob/simulation/output_frame', '/bob/camera/all_sky/bayer')],
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
                name='mask_application_node',
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_usemask_arg'), " == True"])),  
            ),
            #Minimal sensitivity:
            ComposableNode(
                package='bob_image_processing',
                plugin='BackgroundSubtractor',
                name='minimal_background_subtractor_node',
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_arg'), " == 'minimal'" ]))
            ),
            #Low sensitivity:
            ComposableNode(
                package='bob_image_processing',
                plugin='BackgroundSubtractor',
                name='low_background_subtractor_node',
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_arg'), " == 'low'" ]))
            ),
            #Medium sensitivity:
            ComposableNode(
                package='bob_image_processing',
                plugin='BackgroundSubtractor',
                name='medium_background_subtractor_node',
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_arg'), " == 'medium'" ]))
            ),
            #High sensitivity:
            ComposableNode(
                package='bob_image_processing',
                plugin='BackgroundSubtractor',
                name='high_background_subtractor_node',
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_arg'), " == 'high'" ]))
            ),            
            ComposableNode(
                package='bob_tracking',
                plugin='TrackProvider',
                name='track_provider_node',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='bob_recorder',
                plugin='RecordManager',
                name='allsky_recorder_node',
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('enable_recording_arg'), " == True"])),  
            ),
            ComposableNode(
                package='bob_image_processing',
                plugin='AnnotatedFrameProvider',
                name='annotated_frame_provider_node',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Nodes for resizing the image in order to stick it on the network for display
            ComposableNode(
                package='bob_image_processing',
                plugin='FrameResizer',
                name='bayer_frame_resizer_node',
                remappings=[
                    ('bob/resizer/source', 'bob/camera/all_sky/bayer'),
                    ('bob/resizer/target', 'bob/camera/all_sky/bayer/resized')],
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='bob_image_processing',
                plugin='FrameResizer',
                name='foreground_mask_frame_resizer_node',
                remappings=[
                    ('bob/resizer/source', 'bob/frames/all_sky/foreground_mask'),
                    ('bob/resizer/target', 'bob/frames/all_sky/foreground_mask/resized')],
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='bob_image_processing',
                plugin='FrameResizer',
                name='annotated_frame_resizer_node',
                remappings=[
                    ('bob/resizer/source', 'bob/frames/annotated'),
                    ('bob/resizer/target', 'bob/frames/annotated/resized')],
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}]
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
            condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_usemask_arg'), " == True" ])),
            msg=['Masking is set to: ON.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('enable_recording_arg'), " == True" ])),
            msg=['Recording is set to: ON.']),

        kernel_container
        ]
    )