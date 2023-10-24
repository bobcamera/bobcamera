import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the application config
    #config = os.path.join(get_package_share_directory('bob_launch'), 'config', 'app_config.yaml')
    config = 'assets/config/app_config.yaml'

    ros_bridge_package_dir = get_package_share_directory('rosbridge_server')

    mask_webapi_node = Node(
        package='bob_webapi',
        #namespace='bob',
        executable='mask_webapi',
        name='mask_webapi_node',
        parameters = [config],
        remappings=[
            #('bob/observer_frame/source', 'bob/camera/all_sky/bayer/resized')
        ],
    )

    return LaunchDescription([

        mask_webapi_node,

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    ros_bridge_package_dir,
                    'launch/rosbridge_websocket_launch.xml'))
        )
    ])