import os
from launch import LaunchDescription
from launch_ros.actions import Node
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
            ('bob/observer_frame/source', 'bob/camera/all_sky/bayer/resized')],
    )

    cloud_estimator_node = Node(
        package='bob_observer',
        #namespace='bob',
        executable='cloud_estimator',
        name='cloud_estimator_node',
        parameters = [config],
        remappings=[
            ('bob/observer_frame/source', 'bob/camera/all_sky/bayer/resized')],        
    )
    
    tracking_monitor_node = Node(
        package='bob_observer',
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

    return LaunchDescription([
        day_night_classifier_node,
        cloud_estimator_node,
        tracking_monitor_node,
        prometheus_node
    ])