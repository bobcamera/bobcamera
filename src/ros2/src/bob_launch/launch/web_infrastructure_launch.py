import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the application config
    #config = os.path.join(get_package_share_directory('bob_launch'), 'config', 'app_config.yaml')
    config = 'assets/config/app_config.yaml'
    namespace = EnvironmentVariable('BOB_NAMESPACE', default_value="")
    loglevel = EnvironmentVariable('BOB_LOGLEVEL', default_value="INFO")

    ros_bridge_package_dir = get_package_share_directory('rosbridge_server')
    ros_bridge_launch_file = os.path.join(ros_bridge_package_dir, 'launch/rosbridge_websocket_launch.xml')

    info_webapi_node = Node(
        package='bob_webapi',
        namespace=namespace,
        executable='info_webapi',
        name='info_webapi_node',
        arguments=['--ros-args', '--log-level', loglevel],
        parameters = [config]
    )

    mask_webapi_node = Node(
        package='bob_webapi',
        namespace=namespace,
        executable='mask_webapi',
        name='mask_webapi_node',
        parameters = [config],
        arguments=['--ros-args', '--log-level', loglevel],
        remappings=[
            #('input', 'output')
        ],
    )

    ros_bridge = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(ros_bridge_launch_file),
        launch_arguments = {'port': LaunchConfiguration('ros_bridge_port_arg')}.items()
    )

    return LaunchDescription([

        info_webapi_node,
        mask_webapi_node,

        ros_bridge
    ])