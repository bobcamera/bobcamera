import os
import launch
import yaml
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer, Node
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

    video_file1 = '/workspaces/bobcamera/test/fisheye_videos/brad_drone_1.mp4'
    video_file2 = '/workspaces/bobcamera/test/fisheye_videos/Dahua-20220901-184734.mp4'

    simulation_node = Node(            
        package='bob_simulate',
        executable='object_simulator',
        #executable='simulated_video_provider',
        parameters = [
            {"height": LaunchConfiguration('simulation_height_arg')},
            {"width": LaunchConfiguration('simulation_width_arg')},
            {"target_object_diameter": LaunchConfiguration('simulation_target_object_diameter_arg')},
        ],
        remappings=[('bob/simulation/output_frame', '/bob/camera/all_sky/bayer')],
        #arguments=[],
        condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'simulate'" ])),
    )

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
                    ('/ipcamera/image_raw', 'bob/simulation/input_frame'),
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
        condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp_overlay'" ])),
        output='screen',
    )

    rtsp_overlay_simulation_node = Node(            
        package='bob_simulate',
        executable='simulation_overlay_provider',
        parameters = [
            {"height": LaunchConfiguration('simulation_height_arg')},
            {"width": LaunchConfiguration('simulation_width_arg')},
            {"target_object_diameter": LaunchConfiguration('simulation_target_object_diameter_arg')},
        ],
        remappings=[('bob/simulation/output_frame', '/bob/camera/all_sky/bayer')],
        #arguments=[],
        condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp_overlay'" ])),
    )

    return launch.LaunchDescription([

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'simulate'" ])),
            msg=['Source launch argument = SIMULATE source.']),

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp_overlay'" ])),
            msg=['Source launch argument = SIMULATE - RTSP OVERLAY source.']),

        simulation_node,
        rstp_container,
        rtsp_overlay_simulation_node
        ]
    )