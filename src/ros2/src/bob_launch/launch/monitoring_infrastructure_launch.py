from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    day_night_classifier_node = Node(
        package='bob_observer',
        #namespace='bob',
        executable='day_night_classifier',
        name='day_night_classifier',
        parameters=[
            #params,
            {'observer_timer_interval': 30},
            {'observer_day_night_brightness_threshold': 95}],
        remappings=[
            ('bob/observer_frame/source', 'bob/camera/all_sky/bayer/resized')],
    )

    cloud_estimator_node = Node(
        package='bob_observer',
        #namespace='bob',
        executable='cloud_estimator',
        name='cloud_estimator',
        parameters=[
            #params,
            {'observer_timer_interval': 30}],
        remappings=[
            ('bob/observer_frame/source', 'bob/camera/all_sky/bayer/resized')],        
    )
    
    tracking_monitor_node = Node(
        package='bob_observer',
        #namespace='bob',
        executable='tracking_monitor',
        name='tracking_monitor',
        parameters=[
            #params,
            {'observer_tracker_monitor_busy_interval': 5},
            {'observer_tracker_monitor_idle_interval': 60},
            {'observer_tracking_profile_busy_switch_threshold': 5},
            {'observer_tracker_sample_set': 5}],
    )

    prometheus_node = Node(
        package='bob_monitor',
        #namespace='bob',
        executable='prometheus_metrics',
        name='prometheus_metrics'
    )

    return LaunchDescription([
        day_night_classifier_node,
        cloud_estimator_node,
        tracking_monitor_node,
        prometheus_node
    ])