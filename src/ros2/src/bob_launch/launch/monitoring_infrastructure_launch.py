import os
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PythonExpression, LaunchConfiguration 
from launch.conditions import IfCondition
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config = 'assets/config/app_config.yaml'

    pipeline_monitor_container = ComposableNodeContainer(
        name='pipeline_monitor_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[

            ComposableNode(
               package='bob_image_processing',
               plugin='TrackSensitivityMonitor',
               name='track_sensitivity_monitor_node',
               parameters = [config],
               extra_arguments=[{'use_intra_process_comms': True}],
               condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_autotune_arg'), " == True" ]))
            ),
            ComposableNode(
                package='bob_recorder',
                plugin='RecordingMonitor',
                name='allsky_recorder_monitoring_node',
                parameters = [config],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(PythonExpression([LaunchConfiguration('enable_recording_arg'), " == True"])),
            ),               
        ],
        output='screen',
    )

    day_night_classifier_node = Node(
        package='bob_observer',
        #namespace='bob',
        executable='day_night_classifier',
        name='day_night_classifier_node',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters = [config],
        remappings=[
            ('bob/observer_frame/source', 'bob/frames/allsky/masked/detection/resized')],
    )

    cloud_estimator_node = Node(
        package='bob_observer',
        #namespace='bob',
        executable='cloud_estimator',
        name='cloud_estimator_node',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters = [config],
        remappings=[
            ('bob/observer_frame/source', 'bob/frames/allsky/masked/detection/resized')],        
    )
    
    star_mask_node = Node(
        package='bob_observer',
        #namespace='bob',
        executable='star_mask',
        name='star_mask_node',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters = [config],
        remappings=[
            ('bob/observer_frame/source', '/bob/frames/allsky/original/resized')],
        condition=IfCondition(PythonExpression([LaunchConfiguration('enable_star_mask_arg'), " == True"])),
    )
    
    monitoring_status_aggregator_node = Node(
        package='bob_monitor',
        #namespace='bob',
        executable='monitoring_status_aggregator',
        name='monitoring_status_aggregator_node',
        #arguments=['--ros-args', '--log-level', 'DEBUG'],
        parameters = [config],
    )

    prometheus_node = Node(
        package='bob_monitor',
        #namespace='bob',
        executable='prometheus_metrics',
        name='prometheus_metrics_node',
        parameters = [config],
    )
    
    onvif_node = Node(
        package='bob_monitor',
        #namespace='bob',
        executable='onvif_service',
        name='onvif_service_node',
        parameters = [config],
        condition=IfCondition(PythonExpression([
            LaunchConfiguration('source_arg'), 
            " == 'rtsp'",
            " or ", 
            LaunchConfiguration('source_arg'), 
            " == 'rtsp_overlay'"])),
    )

    ptz_manager_node = Node(
        package='bob_monitor',
        #namespace='bob',
        executable='ptz_manager',
        name='ptz_manager_node',
        parameters = [config],
    )

    config_manager_node = Node(
        package='bob_monitor',
        #namespace='bob',
        executable='config_manager',
        name='config_manager_node',
        parameters = [config],
    )

    return LaunchDescription([      

        LogInfo(
            condition=IfCondition(PythonExpression([LaunchConfiguration('tracking_sensitivity_autotune_arg'), " == True" ])),
            msg=['Tracking sensitivity AutoTune enabled.']),

        day_night_classifier_node,
        cloud_estimator_node,
        star_mask_node,
        monitoring_status_aggregator_node,
        pipeline_monitor_container,
        # prometheus_node,
        onvif_node,
        # ptz_manager_node
        # config_manager_node,
    ])