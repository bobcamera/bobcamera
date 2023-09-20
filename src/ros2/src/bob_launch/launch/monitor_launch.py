from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    day_night_classifier_node = Node(
        package='bob_observer',
        #namespace='bob',
        executable='day_night_classifier',
        name='day_night_classifier',
        remappings=[
            ('bob/observer_frame/source', 'bob/camera/all_sky/bayer/resized')],
    )

    cloud_estimator_node = Node(
        package='bob_observer',
        #namespace='bob',
        executable='cloud_estimator',
        name='cloud_estimator',
        remappings=[
            ('bob/observer_frame/source', 'bob/camera/all_sky/bayer/resized')],        
    )
    
    tracking_monitor_node = Node(
        package='bob_observer',
        #namespace='bob',
        executable='tracking_monitor',
        name='tracking_monitor'
    )

    prometheus__node = Node(
        package='bob_monitor',
        #namespace='bob',
        executable='prometheus_metrics',
        name='prometheus_metrics'
    )

    return LaunchDescription([
        day_night_classifier_node,
        cloud_estimator_node,
        tracking_monitor_node,
        prometheus__node   
    ])