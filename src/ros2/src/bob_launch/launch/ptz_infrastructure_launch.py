import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration 
from launch.substitutions import PythonExpression, LaunchConfiguration 
from launch.conditions import IfCondition

def generate_launch_description():

    # Get the application config
    #config = os.path.join(get_package_share_directory('bob_launch'), 'config', 'app_config.yaml')
    config = 'assets/config/app_config.yaml'

    onvif_node = Node(
        package='bob_ptz',
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

    return LaunchDescription([

        onvif_node,
    ])