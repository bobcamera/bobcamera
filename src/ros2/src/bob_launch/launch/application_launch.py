import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    #https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/

    launch_package_dir = get_package_share_directory('bob_launch')
    ros_bridge_package_dir = get_package_share_directory('rosbridge_server')

    source_arg_value = EnvironmentVariable('BOB_SOURCE', default_value="'video'") 
    enable_visualiser_arg_value = EnvironmentVariable('BOB_ENABLE_VISUALISER', default_value="True")
    enable_rosbridge_arg_value = EnvironmentVariable('BOB_ENABLE_ROSBRIDGE', default_value="False")

    source_arg = DeclareLaunchArgument(
        'source_arg',
        default_value=source_arg_value,
        description="Argument for the image source of the application."
        )

    enable_visualiser_arg = DeclareLaunchArgument(
        'enable_visualiser_arg',
        default_value=enable_visualiser_arg_value,
        description="Argument for the enabling of the visualiser screen."
        )

    enable_rosbridge_arg = DeclareLaunchArgument(
        'enable_rosbridge_arg',
        default_value=enable_rosbridge_arg_value,
        description="Argument for the enabling of the ROS Bridge."
        )

    return LaunchDescription([

        source_arg,
        enable_visualiser_arg,
        enable_rosbridge_arg,
        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/kernel_launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/display_launch.py'])
        ),

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([
        #        launch_package_dir, 
        #        '/monitor_launch.py'])
        #),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    ros_bridge_package_dir,
                    'launch/rosbridge_websocket_launch.xml'))
        )

    ])