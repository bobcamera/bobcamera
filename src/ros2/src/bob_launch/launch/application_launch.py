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

    source_arg_value = EnvironmentVariable('BOB_SOURCE', default_value="'video'")
    rtsp_url_arg_value = EnvironmentVariable('BOB_RTSP_URL', default_value="")
    rtsp_width_arg_value = EnvironmentVariable('BOB_RTSP_WIDTH', default_value="")
    rtsp_height_arg_value = EnvironmentVariable('BOB_RTSP_HEIGHT', default_value="")
    camera_id_arg_value = EnvironmentVariable('BOB_CAMERA_ID', default_value="0")
    enable_visualiser_arg_value = EnvironmentVariable('BOB_ENABLE_VISUALISER', default_value="True")
    optimised_arg_value = EnvironmentVariable('BOB_OPTIMISED', default_value="False")
    enable_rosbridge_arg_value = EnvironmentVariable('BOB_ENABLE_ROSBRIDGE', default_value="False")
    simulation_width_arg_value = EnvironmentVariable('BOB_SIMULATION_WIDTH', default_value="1920")
    simulation_height_arg_value = EnvironmentVariable('BOB_SIMULATION_HEIGHT', default_value="1080")
    simulation_target_object_diameter_arg_value = EnvironmentVariable('BOB_SIMULATION_TARGET_OBJ_DIA', default_value="5")
    bgs_algorithm_value = EnvironmentVariable('BOB_BGS_ALGORITHM', default_value="vibe")
    bgs_vibe_params_value = EnvironmentVariable('BOB_VIBE_PARAMS', default_value="{\'threshold\': 50, \'bgSamples\': 20, \'requiredBGSamples\': 2, \'learningRate\': 4}")
    bgs_wmv_params_value = EnvironmentVariable('BOB_WMV_PARAMS', default_value="{\"enableWeight\": true, \"enableThreshold\": true, \"threshold\": 25.0, \"weight1\": 0.5, \"weight2\": 0.3, \"weight3\": 0.2}")
    blob_params_value = EnvironmentVariable('BOB_BLOB_PARAMS', default_value="{\'sizeThreshold\': 7, \'areaThreshold\': 49, \'minDistance\': 40, \'maxBlobs\': 100}")
    #print(f'Generating launch description....')

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

    enable_visualiser_arg = DeclareLaunchArgument(
        'enable_visualiser_arg',
        default_value=enable_visualiser_arg_value,
        description="Argument for the enabling of the visualiser screen."
        )

    optimised_arg = DeclareLaunchArgument(
        'optimised_arg',
        default_value=optimised_arg_value,
        description="Argument for the enabling of the visualiser screen."
        )

    enable_rosbridge_arg = DeclareLaunchArgument(
        'enable_rosbridge_arg',
        default_value=enable_rosbridge_arg_value,
        description="Argument for the enabling of the ROS Bridge."
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

    simulation_target_object_diameter_arg = DeclareLaunchArgument(
        'simulation_target_object_diameter_arg',
        default_value=simulation_target_object_diameter_arg_value,
        description="Simulation target object diameter."
        )
    
    bgs_algorithm_arg = DeclareLaunchArgument(
        'bgs_algorithm_arg',
        default_value=bgs_algorithm_value,
        description="Argument for the Background Subtraction algorithm."
        )
    
    bgs_vibe_params_arg = DeclareLaunchArgument(
        'bgs_vibe_params_arg',
        default_value=bgs_vibe_params_value,
        description="Argument for the Vibe algorithm parameters."
        )

    bgs_wmv_params_arg = DeclareLaunchArgument(
        'bgs_wmv_params_arg',
        default_value=bgs_wmv_params_value,
        description="Argument for the WMV algorithm parameters."
        )
    
    blob_params_arg = DeclareLaunchArgument(
        'blob_params_arg',
        default_value=blob_params_value,
        description="Argument for the blob detector algorithm parameters."
        )

    return LaunchDescription([

        source_arg,

        rtsp_url_arg,
        rtsp_width_arg,
        rtsp_height_arg,
        
        simulation_width_arg,
        simulation_height_arg,
        simulation_target_object_diameter_arg,

        camera_id_arg,
        enable_visualiser_arg,
        optimised_arg,
        enable_rosbridge_arg,

        bgs_algorithm_arg,
        bgs_vibe_params_arg,
        bgs_wmv_params_arg,
        blob_params_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/simulation_launch.py']),
        ),

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/monitor_launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                launch_package_dir, 
                '/rosbridge_launch.py'])
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