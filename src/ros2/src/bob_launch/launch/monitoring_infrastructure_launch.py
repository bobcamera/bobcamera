import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PythonExpression, LaunchConfiguration 
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    #config = os.path.join(get_package_share_directory('bob_launch'), 'config', 'app_config.yaml')
    config = 'assets/config/app_config.yaml'

    day_night_classifier_node = Node(
        package='bob_observer',
        #namespace='bob',
        executable='day_night_classifier',
        name='day_night_classifier_node',
        parameters = [config],
        remappings=[
            ('bob/observer_frame/source', 'bob/frames/masked/resized')],
    )

    cloud_estimator_node = Node(
        package='bob_observer',
        #namespace='bob',
        executable='cloud_estimator',
        name='cloud_estimator_node',
        parameters = [config],
        remappings=[
            ('bob/observer_frame/source', 'bob/frames/masked/resized')],        
    )
    
    tracking_monitor_node = Node(
        package='bob_monitor',
        #namespace='bob',
        executable='tracking_monitor',
        name='tracking_monitor_node',
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
        # day_night_classifier_node,
        # cloud_estimator_node,
        tracking_monitor_node,
        # prometheus_node,
        # onvif_node,
        # ptz_manager_node
        config_manager_node,
    ])