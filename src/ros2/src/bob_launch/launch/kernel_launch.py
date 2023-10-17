import os
import launch
import yaml
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PythonExpression, LaunchConfiguration 
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the config directory
    config_dir = os.path.join(get_package_share_directory('bob_launch'), 'config')
    #param_config = os.path.join(config_dir, "ipcamera.yaml")
    #with open(param_config, 'r') as f:
    #    params = yaml.safe_load(f)["ipcamera"]["ros__parameters"]

    # Alternatively can use "package://" as discussed:
    # https://answers.ros.org/question/333521/ros2-url-to-camera_info-yaml-not-being-recognized/
    config_file = 'file://' + os.path.join(config_dir, "camera_info.yaml")

    video_file1 = '/workspaces/bobcamera/test/fisheye_videos/mike_drone.mp4'
    video_file2 = '/workspaces/bobcamera/test/fisheye_videos/Dahua-20220901-184734.mp4'
    video_file3 = '/workspaces/bobcamera/test/fisheye_videos/brad_drone_1.mp4'
    

    processing_pipeline_container = ComposableNodeContainer(
        name='track_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[

            ComposableNode(
               package='bob_camera',
               plugin='IPCamera',
               name='ipcamera',
               remappings=[
                   ('/ipcamera/image_raw', 'bob/camera/all_sky/bayer'),
                   ('/ipcamera/camera_info', 'bob/camera/all_sky/camera_info')],
               parameters=[
                   #params,
                   {'rtsp_uri': LaunchConfiguration('rtsp_url_arg')},
                   {'image_topic': 'image_raw'},
                   {'image_width': LaunchConfiguration('rtsp_width_arg')},
                   {'image_height': LaunchConfiguration('rtsp_height_arg')},
                   {'camera_calibration_file': config_file}],
               extra_arguments=[{'use_intra_process_comms': True}],
               condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp'" ]))
            ),
            ComposableNode(
               package='bob_camera',
               plugin='IPCamera',
               name='ipcamera',
               remappings=[
                   ('/ipcamera/image_raw', 'bob/simulation/input_frame'),
                   ('/ipcamera/camera_info', 'bob/camera/all_sky/camera_info')],
               parameters=[
                   #params,
                   {'rtsp_uri': LaunchConfiguration('rtsp_url_arg')},
                   {'image_topic': 'image_raw'},
                   {'image_width': LaunchConfiguration('rtsp_width_arg')},
                   {'image_height': LaunchConfiguration('rtsp_height_arg')},
                   {'camera_calibration_file': config_file}],
               extra_arguments=[{'use_intra_process_comms': True}],
               condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp_overlay'" ]))
            ),
            ComposableNode(
                package='bob_camera',
                plugin='WebCameraVideo',
                name='web_camera_video_node',
                parameters=[{'image_publish_topic': 'bob/camera/all_sky/bayer'}
                    , {'image_info_publish_topic': 'bob/camera/all_sky/image_info'}
                    , {'camera_info_publish_topic': 'bob/camera/all_sky/camera_info'}
                    , {'is_video': True}
                    , {'videos': [video_file1, video_file2, video_file3]}
                    , {'resize_height': 0}],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'video'" ])),
            ),
            ComposableNode(
                package='bob_camera',
                plugin='WebCameraVideo',
                name='usb_camera_node',
                parameters=[{'image_publish_topic': 'bob/camera/all_sky/bayer'}
                    , {'image_info_publish_topic': 'bob/camera/all_sky/image_info'}
                    , {'camera_info_publish_topic': 'bob/camera/all_sky/camera_info'}
                    , {'is_video': False}
                    , {'camera_id': LaunchConfiguration('camera_id_arg')}],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'usb'" ])),
            ),
            ComposableNode(
                package='bob_simulator', 
                plugin='MovingObjectsSimulation', 
                name='simulated_frame_provider_node',  
                parameters=[{'image_publish_topic': 'bob/simulation/output_frame'} 
                    , {'height': LaunchConfiguration('simulation_height_arg')}
                    , {'width': LaunchConfiguration('simulation_width_arg')}
                    , {'num_objects': LaunchConfiguration('simulation_num_objects_arg')}
                    ],
                remappings=[
                    ('bob/simulation/output_frame', 'bob/camera/all_sky/bayer')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'simulate'" ])),  
            ) , 
            ComposableNode(
                package='bob_camera',
                plugin='WebCameraVideo',
                name='web_camera_video_node',
                parameters=[{'image_publish_topic': 'bob/simulation/input_frame'}
                    , {'image_info_publish_topic': 'bob/camera/all_sky/image_info'}
                    , {'camera_info_publish_topic': 'bob/camera/all_sky/camera_info'}
                    , {'is_video': True}
                    , {'videos': [video_file1, video_file2, video_file3]}
                    , {'resize_height': 0}],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'video_overlay'" ])),
            ),
            ComposableNode(
                package='bob_simulator',
                plugin='SimulationOverlayProviderNode',  
                name='simulation_overlay_provider_node',
                parameters=[
                    {"height": LaunchConfiguration('simulation_height_arg')},
                    {"width": LaunchConfiguration('simulation_width_arg')},
                    {"num_objects": LaunchConfiguration('simulation_num_objects_arg')}
                ],
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
                parameters=[{'mask_file': LaunchConfiguration('tracking_maskfile_arg')}],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_usemask_arg'), " == True"])),  
            ),
            #Minimal sensitivity:
            ComposableNode(
                package='bob_image_processing',
                plugin='BackgroundSubtractor',
                name='background_subtractor_node',
                parameters=[{'bgs': LaunchConfiguration('bgs_algorithm_arg')}
                    , {'vibe_params': "{\"threshold\": 70, \"bgSamples\": 32, \"requiredBGSamples\": 1, \"learningRate\": 4}"}
                    , {'wmv_params': "{\"enableWeight\": true, \"enableThreshold\": true, \"threshold\": 60.0, \"weight1\": 0.5, \"weight2\": 0.3, \"weight3\": 0.2}"}
                    , {'blob_params': "{\"sizeThreshold\": 11, \"areaThreshold\": 121, \"minDistance\": 121, \"maxBlobs\": 50}"}
                    , {'use_mask': LaunchConfiguration('tracking_usemask_arg')}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_arg'), " == 'minimal'" ]))
            ),
            #Low sensitivity:
            ComposableNode(
                package='bob_image_processing',
                plugin='BackgroundSubtractor',
                name='background_subtractor_node',
                parameters=[{'bgs': LaunchConfiguration('bgs_algorithm_arg')}
                    , {'vibe_params': "{\"threshold\": 55, \"bgSamples\": 32, \"requiredBGSamples\": 1, \"learningRate\": 4}"}
                    , {'wmv_params': "{\"enableWeight\": true, \"enableThreshold\": true, \"threshold\": 45.0, \"weight1\": 0.5, \"weight2\": 0.3, \"weight3\": 0.2}"}
                    , {'blob_params': "{\"sizeThreshold\": 8, \"areaThreshold\": 64, \"minDistance\": 64, \"maxBlobs\": 50}"}
                    , {'use_mask': LaunchConfiguration('tracking_usemask_arg')}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_arg'), " == 'low'" ]))
            ),
            #Medium sensitivity:
            ComposableNode(
                package='bob_image_processing',
                plugin='BackgroundSubtractor',
                name='background_subtractor_node',
                parameters=[{'bgs': LaunchConfiguration('bgs_algorithm_arg')}
                    , {'vibe_params': "{\"threshold\": 40, \"bgSamples\": 16, \"requiredBGSamples\": 1, \"learningRate\": 8}"}
                    , {'wmv_params': "{\"enableWeight\": true, \"enableThreshold\": true, \"threshold\": 30.0, \"weight1\": 0.5, \"weight2\": 0.3, \"weight3\": 0.2}"}
                    , {'blob_params': "{\"sizeThreshold\": 4, \"areaThreshold\": 16, \"minDistance\": 16, \"maxBlobs\": 50}"}
                    , {'use_mask': LaunchConfiguration('tracking_usemask_arg')}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_arg'), " == 'medium'" ]))
            ),
            #High sensitivity:
            ComposableNode(
                package='bob_image_processing',
                plugin='BackgroundSubtractor',
                name='background_subtractor_node',
                parameters=[{'bgs': LaunchConfiguration('bgs_algorithm_arg')}
                    , {'vibe_params': "{\"threshold\": 30, \"bgSamples\": 16, \"requiredBGSamples\": 1, \"learningRate\": 8}"}
                    , {'wmv_params': "{\"enableWeight\": true, \"enableThreshold\": true, \"threshold\": 15.0, \"weight1\": 0.5, \"weight2\": 0.3, \"weight3\": 0.2}"}
                    , {'blob_params': "{\"sizeThreshold\": 2, \"areaThreshold\": 4, \"minDistance\": 4, \"maxBlobs\": 50}"}
                    , {'use_mask': LaunchConfiguration('tracking_usemask_arg')}
                ],
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
                plugin='VideoRecorder',
                name='allsky_recorder_node',
                parameters=[{'video_directory': 'recordings/allsky'}
                    , {'img_topic': 'bob/camera/all_sky/bayer'}
                    , {'tracking_topic': 'bob/tracker/tracking'}
                    , {'prefix': ''}
                    , {'codec': 'X264'}
                    , {'video_fps': 15.0}
                    , {'seconds_save': 2}
                ],                
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('enable_recording_arg'), " == True"])),  
            ),
            ComposableNode(
                package='bob_recorder',
                plugin='VideoRecorder',
                name='foreground_mask_recorder_node',
                parameters=[{'video_directory': 'recordings/foreground_mask'}
                    , {'img_topic': 'bob/frames/all_sky/foreground_mask'}
                    , {'tracking_topic': 'bob/tracker/tracking'}
                    , {'prefix': ''}
                    , {'codec': 'X264'}
                    , {'video_fps': 15.0}
                    , {'seconds_save': 2}
                ],                
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
                parameters=[{'resize_height': 960}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='bob_image_processing',
                plugin='FrameResizer',
                name='foreground_mask_frame_resizer_node',
                remappings=[
                    ('bob/resizer/source', 'bob/frames/all_sky/foreground_mask'),
                    ('bob/resizer/target', 'bob/frames/all_sky/foreground_mask/resized')],
                parameters=[{'resize_height': 960}],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('optimised_arg'), " == False" ])),
            ),
            ComposableNode(
                package='bob_image_processing',
                plugin='FrameResizer',
                name='annotated_frame_resizer_node',
                remappings=[
                    ('bob/resizer/source', 'bob/frames/annotated'),
                    ('bob/resizer/target', 'bob/frames/annotated/resized')],
                parameters=[{'resize_height': 960}],
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

        processing_pipeline_container
        ]
    )