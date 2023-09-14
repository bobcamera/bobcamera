import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ros_bridge_package_dir = get_package_share_directory('rosbridge_server')

    return LaunchDescription([

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    ros_bridge_package_dir,
                    'launch/rosbridge_websocket_launch.xml'))
        )
    ])