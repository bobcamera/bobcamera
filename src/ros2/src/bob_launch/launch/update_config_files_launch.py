import os
import yaml
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration 
from ament_index_python.packages import get_package_share_directory

def application_config(context):

    video_file1 = '/workspaces/bobcamera/test/fisheye_videos/mike_drone.mp4'
    video_file2 = '/workspaces/bobcamera/test/fisheye_videos/Dahua-20220901-184734.mp4'
    video_file3 = '/workspaces/bobcamera/test/fisheye_videos/brad_drone_1.mp4'

    # get the values provided as part of the launch arguments
    rtsp_url = LaunchConfiguration('rtsp_url_arg').perform(context)
    videos = [video_file1, video_file2, video_file3]
    image_width = int(LaunchConfiguration('rtsp_width_arg').perform(context))
    image_height = int(LaunchConfiguration('rtsp_height_arg').perform(context))
    camera_id = int(LaunchConfiguration('camera_id_arg').perform(context))

    simulation_height = int(LaunchConfiguration('simulation_height_arg').perform(context))
    simulation_width = int(LaunchConfiguration('simulation_width_arg').perform(context))
    simulation_num_objects = int(LaunchConfiguration('simulation_num_objects_arg').perform(context))

    bgs_algo = str(LaunchConfiguration('bgs_algorithm_arg').perform(context))

    tracking_mask_file = str(LaunchConfiguration('tracking_maskfile_arg').perform(context))
    tracking_use_mask = LaunchConfiguration('tracking_usemask_arg').perform(context) in ('True', 'true')
    tracking_mask_dir = os.path.dirname(tracking_mask_file)

    # Get the config directory
    config_dir = os.path.join(get_package_share_directory('bob_launch'), 'config')
    camera_config_file = 'file://' + os.path.join(config_dir, "camera_info.yaml")
    app_config_file = os.path.join(config_dir, "app_config.yaml")

    with open(app_config_file, 'r') as read:
        yaml_output = yaml.safe_load(read)

        # rstp_camera_node
        yaml_output['rstp_camera_node']['ros__parameters']['rtsp_uri'] = rtsp_url
        yaml_output['rstp_camera_node']['ros__parameters']['image_width'] = image_width
        yaml_output['rstp_camera_node']['ros__parameters']['image_height'] = image_height
        yaml_output['rstp_camera_node']['ros__parameters']['camera_calibration_file'] = camera_config_file
    
        # rstp_overlay_camera_node
        yaml_output['rstp_overlay_camera_node']['ros__parameters']['rtsp_uri'] = rtsp_url
        yaml_output['rstp_overlay_camera_node']['ros__parameters']['image_width'] = image_width
        yaml_output['rstp_overlay_camera_node']['ros__parameters']['image_height'] = image_height
        yaml_output['rstp_overlay_camera_node']['ros__parameters']['camera_calibration_file'] = camera_config_file

        # web_camera_video_node
        yaml_output['web_camera_video_node']['ros__parameters']['videos'] = videos

        # web_camera_video_node
        yaml_output['usb_camera_node']['ros__parameters']['camera_id'] = camera_id

        # simulated_frame_provider_node
        yaml_output['simulated_frame_provider_node']['ros__parameters']['num_objects'] = simulation_num_objects
        yaml_output['simulated_frame_provider_node']['ros__parameters']['height'] = simulation_height
        yaml_output['simulated_frame_provider_node']['ros__parameters']['width'] = simulation_width

        # web_camera_video_overlay_node
        yaml_output['web_camera_video_overlay_node']['ros__parameters']['videos'] = videos

        # simulation_overlay_provider_node
        yaml_output['simulation_overlay_provider_node']['ros__parameters']['num_objects'] = simulation_num_objects
        yaml_output['simulation_overlay_provider_node']['ros__parameters']['height'] = simulation_height
        yaml_output['simulation_overlay_provider_node']['ros__parameters']['width'] = simulation_width

        # mask_application_node
        yaml_output['mask_application_node']['ros__parameters']['mask_file'] = tracking_mask_file

        # minimal_background_subtractor_node
        yaml_output['minimal_background_subtractor_node']['ros__parameters']['bgs'] = bgs_algo
        yaml_output['minimal_background_subtractor_node']['ros__parameters']['use_mask'] = tracking_use_mask

        # low_background_subtractor_node
        yaml_output['low_background_subtractor_node']['ros__parameters']['bgs'] = bgs_algo
        yaml_output['low_background_subtractor_node']['ros__parameters']['use_mask'] = tracking_use_mask

        # medium_background_subtractor_node
        yaml_output['medium_background_subtractor_node']['ros__parameters']['bgs'] = bgs_algo
        yaml_output['medium_background_subtractor_node']['ros__parameters']['use_mask'] = tracking_use_mask

        # high_background_subtractor_node
        yaml_output['high_background_subtractor_node']['ros__parameters']['bgs'] = bgs_algo
        yaml_output['high_background_subtractor_node']['ros__parameters']['use_mask'] = tracking_use_mask

        # mask_webapi_node
        yaml_output['mask_webapi_node']['ros__parameters']['masks_folder'] = tracking_mask_dir
        yaml_output['mask_webapi_node']['ros__parameters']['width'] = image_width
        yaml_output['mask_webapi_node']['ros__parameters']['height'] = image_height

    # Update the camera_info file with the provided launch arguments
    with open(app_config_file, 'w') as write:
        yaml.Dumper.ignore_aliases = lambda *args: True
        yaml.dump(yaml_output, write, sort_keys = False, width=1080)

def camera_info_config(context):

    # get the values provided as part of the launch arguments
    image_width = int(LaunchConfiguration('rtsp_width_arg').perform(context))
    image_height = int(LaunchConfiguration('rtsp_height_arg').perform(context))

    # Get the config directory
    config_dir = os.path.join(get_package_share_directory('bob_launch'), 'config')
    config_file = os.path.join(config_dir, "camera_info.yaml")
    with open(config_file, 'r') as read:
        yaml_output = yaml.safe_load(read)
        yaml_output['image_width'] = image_width
        yaml_output['image_height'] = image_height
    
    # Update the camera_info file with the provided launch arguments
    with open(config_file, 'w') as write:
        yaml.dump(yaml_output, write, sort_keys = False)

def generate_launch_description():
    application_config_func = OpaqueFunction(function = application_config)
    camera_info_config_func = OpaqueFunction(function = camera_info_config)
    #opaqueFunction.condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp' or 'rtsp_overlay'" ]))

    return LaunchDescription([
        application_config_func,
        camera_info_config_func
    ])




