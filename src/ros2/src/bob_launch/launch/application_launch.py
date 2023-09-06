from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    launch_package_dir = get_package_share_directory('bob_launch')

    return LaunchDescription([

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([
        #        launch_package_dir, 
        #        '/video_kernel_launch.py'])
        #),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/rtsp_kernel_launch.py'])
        ),        

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([
        #        launch_package_dir, 
        #        '/usb_kernel_launch.py'])
        #),  

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
    ])