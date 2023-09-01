from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([

        Node(
            package='bob_observer',
            #namespace='bob',
            executable='cloud_estimator',
            name='cloud_estimator'
        ),
        Node(
            package='bob_observer',
            #namespace='bob',
            executable='day_night_classifier',
            name='day_night_classifier'
        ),
        Node(
            package='bob_monitor',
            #namespace='bob',
            executable='prometheus_metrics',
            name='prometheus_metrics'
        ),        
    ])