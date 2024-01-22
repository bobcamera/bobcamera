import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, LogInfo, TimerAction
from launch.substitutions import EnvironmentVariable, LocalSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    #https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/

    launch_package_dir = get_package_share_directory('bob_launch')

    update_config_from_env_vars_arg_value = EnvironmentVariable('BOB_UPDATE_CONFIG_FROM_ENV_VARS', default_value="True")

    source_arg_value = EnvironmentVariable('BOB_SOURCE', default_value="'video'")
    
    rtsp_url_arg_value = EnvironmentVariable('BOB_RTSP_URL', default_value="")
    rtsp_width_arg_value = EnvironmentVariable('BOB_RTSP_WIDTH', default_value="")
    rtsp_height_arg_value = EnvironmentVariable('BOB_RTSP_HEIGHT', default_value="")
    
    camera_id_arg_value = EnvironmentVariable('BOB_CAMERA_ID', default_value="0")    
    fps_arg_value = EnvironmentVariable('BOB_FPS', default_value="15")
    
    enable_visualiser_arg_value = EnvironmentVariable('BOB_ENABLE_VISUALISER', default_value="True")
    optimised_arg_value = EnvironmentVariable('BOB_OPTIMISED', default_value="False")
    
    enable_rosbridge_arg_value = EnvironmentVariable('BOB_ENABLE_ROSBRIDGE', default_value="False")
    enable_recording_arg_value = EnvironmentVariable('BOB_ENABLE_RECORDING', default_value="False")    
    
    simulation_width_arg_value = EnvironmentVariable('BOB_SIMULATION_WIDTH', default_value="1920")
    simulation_height_arg_value = EnvironmentVariable('BOB_SIMULATION_HEIGHT', default_value="1080")
    simulation_num_objects_arg_value = EnvironmentVariable('BOB_SIMULATION_NUM_OBJECTS', default_value="5")    
    
    bgs_algorithm_value = EnvironmentVariable('BOB_BGS_ALGORITHM', default_value="vibe")    
    
    tracking_sensitivity_arg_value  = EnvironmentVariable('BOB_TRACKING_SENSITIVITY', default_value="'high'")
    tracking_usemask_arg_value  = EnvironmentVariable('BOB_TRACKING_USEMASK', default_value="False")
    tracking_maskfile_arg_value  = EnvironmentVariable('BOB_TRACKING_MASK_FILE', default_value="'mask.pgm'")

    video_arg_value  = EnvironmentVariable('BOB_VIDEOS', default_value="")

    #print(f'Generating launch description....')

    update_config_from_env_vars_arg = DeclareLaunchArgument(
        'update_config_from_env_vars_arg',
        default_value=update_config_from_env_vars_arg_value,
        description="Update application config file from environment variables."
        )

    source_arg = DeclareLaunchArgument(
        'source_arg',
        default_value=source_arg_value,
        description="Argument for the image source of the application."
        )

    rtsp_url_arg = DeclareLaunchArgument(
        'rtsp_url_arg',
        default_value=rtsp_url_arg_value,
        description="RTSP url."
        )

    rtsp_width_arg = DeclareLaunchArgument(
        'rtsp_width_arg',
        default_value=rtsp_width_arg_value,
        description="RTSP image width."
        )
    
    rtsp_height_arg = DeclareLaunchArgument(
        'rtsp_height_arg',
        default_value=rtsp_height_arg_value,
        description="RTSP image height."
        )

    camera_id_arg = DeclareLaunchArgument(
        'camera_id_arg',
        default_value=camera_id_arg_value,
        description="USB Camera Id."
        )

    fps_arg = DeclareLaunchArgument(
        'fps_arg',
        default_value=fps_arg_value,
        description="FPS of the frames being processed."
        )

    enable_visualiser_arg = DeclareLaunchArgument(
        'enable_visualiser_arg',
        default_value=enable_visualiser_arg_value,
        description="Argument for the enabling of the visualiser screen."
        )

    optimised_arg = DeclareLaunchArgument(
        'optimised_arg',
        default_value=optimised_arg_value,
        description="Argument for the enabling optimisation i.e. only list annotated frame in visualiser."
        )

    enable_rosbridge_arg = DeclareLaunchArgument(
        'enable_rosbridge_arg',
        default_value=enable_rosbridge_arg_value,
        description="Argument for the enabling of the ROS Bridge."
        )

    enable_recording_arg = DeclareLaunchArgument(
        'enable_recording_arg',
        default_value=enable_recording_arg_value,
        description="Argument for the enabling of the recorder."
        )

    simulation_width_arg = DeclareLaunchArgument(
        'simulation_width_arg',
        default_value=simulation_width_arg_value,
        description="Simulation image width."
        )
    
    simulation_height_arg = DeclareLaunchArgument(
        'simulation_height_arg',
        default_value=simulation_height_arg_value,
        description="Simulation image height."
        )

    simulation_num_objects_arg = DeclareLaunchArgument(
        'simulation_num_objects_arg',
        default_value=simulation_num_objects_arg_value,
        description="Simulation number of objects."
        )
    
    bgs_algorithm_arg = DeclareLaunchArgument(
        'bgs_algorithm_arg',
        default_value=bgs_algorithm_value,
        description="Argument for the Background Subtraction algorithm."
        )
    
    tracking_sensitivity_arg = DeclareLaunchArgument(
        'tracking_sensitivity_arg',
        default_value=tracking_sensitivity_arg_value,
        description="Tracking sensitivity of the bgs and blob detector, it drives a set of parameters for the algos."
        )  

    tracking_usemask_arg = DeclareLaunchArgument(
        'tracking_usemask_arg',
        default_value=tracking_usemask_arg_value,
        description="Use mask set."
        )    
    
    tracking_maskfile_arg = DeclareLaunchArgument(
        'tracking_maskfile_arg',
        default_value=tracking_maskfile_arg_value,
        description="Use mask set."
        )   

    video_arg = DeclareLaunchArgument(
        'video_arg',
        default_value=video_arg_value,
        description="Videos to use as playback."
        )  

    return LaunchDescription([

        update_config_from_env_vars_arg,

        source_arg,

        rtsp_url_arg,
        rtsp_width_arg,
        rtsp_height_arg,
        
        simulation_width_arg,
        simulation_height_arg,
        simulation_num_objects_arg,

        camera_id_arg,
        fps_arg,

        enable_visualiser_arg,
        optimised_arg,
        enable_rosbridge_arg,
        enable_recording_arg,

        bgs_algorithm_arg,
        tracking_sensitivity_arg,
        tracking_usemask_arg,
        tracking_maskfile_arg,

        video_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/update_config_files_launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/kernel_launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/display_infrastructure_launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/monitoring_infrastructure_launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/web_infrastructure_launch.py'])
        ),

        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Application Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),

    ])