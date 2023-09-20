import os
import launch
import yaml
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PythonExpression, LaunchConfiguration 
from launch_ros.parameter_descriptions import ParameterValue
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
    
    rtsp_container = ComposableNodeContainer(
        name='rtsp_container',
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
                    #params,
                    {'rtsp_uri': LaunchConfiguration('rtsp_url_arg')},
                    {'image_topic': 'image_raw'},
                    {'image_width': LaunchConfiguration('rtsp_width_arg')},
                    {'image_height': LaunchConfiguration('rtsp_height_arg')},
                    {'camera_calibration_file': config_file}],
                extra_arguments=[{'use_intra_process_comms': True}]),
        ],
        condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp'" ])),
        output='screen',
    )

    processing_pipeline_container = ComposableNodeContainer(
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
                package='bob_image_processing',
                plugin='BackgroundSubtractor',
                name='background_subtractor_node',
                parameters=[{'bgs': LaunchConfiguration('bgs_algorithm_arg')}
                    , {'vibe_params': ParameterValue(LaunchConfiguration('bgs_vibe_params_arg'), value_type=str)}
                    , {'wmv_params': ParameterValue(LaunchConfiguration('bgs_wmv_params_arg'), value_type=str)}
                    , {'blob_params': ParameterValue(LaunchConfiguration('blob_params_arg'), value_type=str)}
                ],
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
                extra_arguments=[{'use_intra_process_comms': True}]),  
            ComposableNode(
                package='bob_simulate', 
                plugin='ObjectSimulator', 
                name='simulated_frame_provider_node',  
                # parameters=[],  # Any parameters you might have
                # remappings=[
                #     ('bob/object_simulator/frame', 'bob/camera/all_sky/bayer')
                # ],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'simulate'" ])),  # New source_arg value for the simulator
            )                  
        ],
        output='screen',
    )    

    return launch.LaunchDescription([

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'video'" ])),
            msg=['Source launch argument = VIDEO source.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp'" ])),
            msg=['Source launch argument = RTSP source.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'usb'" ])),
            msg=['Source launch argument = USB source.']),

        rtsp_container,
        processing_pipeline_container
        ]
    )