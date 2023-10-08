import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ros_bridge_package_dir = get_package_share_directory('rosbridge_server')

    mask_webapi_node = Node(
        package='bob_webapi',
        #namespace='bob',
        executable='mask_webapi',
        name='mask_webapi',
        parameters=[
            #params,
            #{'observer_timer_interval': 30},
            #{'observer_day_night_brightness_threshold': 95}
        ],
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