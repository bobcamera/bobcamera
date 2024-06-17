import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, LogInfo, TimerAction
from launch.substitutions import EnvironmentVariable, LocalSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    launch_package_dir = get_package_share_directory('bob_launch')
    
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/allsky_kernel_launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/display_infrastructure_launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/monitoring_infrastructure_launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/web_infrastructure_launch.py']),
        ),
    ])