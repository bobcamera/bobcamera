import os
import yaml
from launch.actions import LogInfo
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration 
from ament_index_python.packages import get_package_share_directory

from onvif2 import ONVIFCamera

def create_storage_folders(context):
    # TODO: Parameterise these a bit better, or drive them from the UI or something but for now
    # hard coding them will have to do.
    os.makedirs('assets', exist_ok=True)
    os.makedirs('assets/config', exist_ok=True)
    os.makedirs('assets/recordings', exist_ok=True)
    os.makedirs('assets/masks', exist_ok=True)

def application_config(context):

    app_config_file_src = os.path.join(get_package_share_directory('bob_launch'), 'config', 'app_config.yaml')
    app_config_file = 'assets/config/app_config.yaml'

    # get the values provided as part of the launch arguments
    source = str(LaunchConfiguration('source_arg').perform(context))
    rtsp_url = str(LaunchConfiguration('rtsp_url_arg').perform(context))
    image_width = int(LaunchConfiguration('rtsp_width_arg').perform(context))
    image_height = int(LaunchConfiguration('rtsp_height_arg').perform(context))
    camera_id = int(LaunchConfiguration('camera_id_arg').perform(context))
    fps = float(LaunchConfiguration('fps_arg').perform(context))

    simulation_height = int(LaunchConfiguration('simulation_height_arg').perform(context))
    simulation_width = int(LaunchConfiguration('simulation_width_arg').perform(context))
    simulation_num_objects = int(LaunchConfiguration('simulation_num_objects_arg').perform(context))

    bgs_algo = str(LaunchConfiguration('bgs_algorithm_arg').perform(context))

    tracking_mask_file = str(LaunchConfiguration('tracking_maskfile_arg').perform(context))
    tracking_use_mask = LaunchConfiguration('tracking_usemask_arg').perform(context) in ('True', 'true')
    tracking_mask_dir = os.path.dirname(tracking_mask_file)

    videos = str(LaunchConfiguration('video_arg').perform(context))

    update_config = LaunchConfiguration('update_config_from_env_vars_arg').perform(context) in ('True', 'true')

    if update_config:
        
        with open(app_config_file_src, 'r') as read:
            yaml_output = yaml.safe_load(read)

            # rtsp_camera_node
            yaml_output['rtsp_camera_node']['ros__parameters']['rtsp_uri'] = rtsp_url
            yaml_output['rtsp_camera_node']['ros__parameters']['image_width'] = image_width
            yaml_output['rtsp_camera_node']['ros__parameters']['image_height'] = image_height
        
            # rtsp_overlay_camera_node
            yaml_output['rtsp_overlay_camera_node']['ros__parameters']['rtsp_uri'] = rtsp_url
            yaml_output['rtsp_overlay_camera_node']['ros__parameters']['image_width'] = image_width
            yaml_output['rtsp_overlay_camera_node']['ros__parameters']['image_height'] = image_height

            # web_camera_video_node
            yaml_output['usb_camera_node']['ros__parameters']['camera_id'] = camera_id

            # web_camera_video_node
            yaml_output['web_camera_video_node']['ros__parameters']['videos'] = videos.split(";")

            # web_camera_video_overlay_node
            yaml_output['web_camera_video_overlay_node']['ros__parameters']['videos'] = videos.split(";")

            # simulated_frame_provider_node
            yaml_output['simulated_frame_provider_node']['ros__parameters']['num_objects'] = simulation_num_objects
            yaml_output['simulated_frame_provider_node']['ros__parameters']['height'] = simulation_height
            yaml_output['simulated_frame_provider_node']['ros__parameters']['width'] = simulation_width
            yaml_output['simulated_frame_provider_node']['ros__parameters']['video_fps'] = fps

            # simulation_overlay_provider_node
            yaml_output['simulation_overlay_provider_node']['ros__parameters']['num_objects'] = simulation_num_objects
            yaml_output['simulation_overlay_provider_node']['ros__parameters']['height'] = simulation_height
            yaml_output['simulation_overlay_provider_node']['ros__parameters']['width'] = simulation_width
            yaml_output['simulation_overlay_provider_node']['ros__parameters']['video_fps'] = fps

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

            # info_webapi_node
            yaml_output['info_webapi_node']['ros__parameters']['frame_width'] = image_width
            yaml_output['info_webapi_node']['ros__parameters']['frame_height'] = image_height
            yaml_output['info_webapi_node']['ros__parameters']['video_fps'] = fps

            # allsky_recorder_node
            yaml_output['allsky_recorder_node']['ros__parameters']['video_fps'] = fps

            if source in ('\'rtsp\'', '\'rtsp_overlay\''):
                (onvif_success, rtsp_user, rtsp_password, rtsp_host, rtsp_port) = get_onvif_config(rtsp_url)
                if onvif_success:
                    yaml_output['rtsp_camera_node']['ros__parameters']['onvif_user'] = rtsp_user
                    yaml_output['rtsp_camera_node']['ros__parameters']['onvif_password'] = rtsp_password
                    yaml_output['rtsp_camera_node']['ros__parameters']['onvif_host'] = rtsp_host
                    yaml_output['rtsp_camera_node']['ros__parameters']['onvif_port'] = rtsp_port

                    yaml_output['rtsp_overlay_camera_node']['ros__parameters']['onvif_user'] = rtsp_user
                    yaml_output['rtsp_overlay_camera_node']['ros__parameters']['onvif_password'] = rtsp_password
                    yaml_output['rtsp_overlay_camera_node']['ros__parameters']['onvif_host'] = rtsp_host
                    yaml_output['rtsp_overlay_camera_node']['ros__parameters']['onvif_port'] = rtsp_port                    

        # Update the camera_info file with the provided launch arguments
        with open(app_config_file, 'w') as write:
            yaml.Dumper.ignore_aliases = lambda *args: True
            yaml.dump(yaml_output, write, sort_keys = False, width=1080)

def camera_config(context):

    # get the values provided as part of the launch arguments
    image_width = int(LaunchConfiguration('rtsp_width_arg').perform(context))
    image_height = int(LaunchConfiguration('rtsp_height_arg').perform(context))

    camera_config_file_src = os.path.join(get_package_share_directory('bob_launch'), 'config', 'camera_info.yaml')
    camera_config_file = 'assets/config/camera_info.yaml'

    with open(camera_config_file_src, 'r') as read:
        yaml_output = yaml.safe_load(read)
        yaml_output['image_width'] = image_width
        yaml_output['image_height'] = image_height
    
    # Update the camera_info file with the provided launch arguments
    with open(camera_config_file, 'w') as write:
        yaml.dump(yaml_output, write, sort_keys = False)

def generate_launch_description():
    create_storage_folders_func = OpaqueFunction(function = create_storage_folders)
    application_config_func = OpaqueFunction(function = application_config)
    camera_info_config_func = OpaqueFunction(function = camera_config)

    return LaunchDescription([
        LogInfo(msg=['Updating config files, this might take a minute, please wait...']),
        create_storage_folders_func,
        application_config_func,
        camera_info_config_func,
        LogInfo(msg=['Config files update complete.']),
    ])

def get_onvif_config(rtsp_url):

    try:
        credstr = rtsp_url[rtsp_url.index("//"):rtsp_url.index("@")]
        if credstr.startswith("//"):
            credstr = credstr.replace("//", "")

        cred_array = credstr.split(":")

        hoststr = rtsp_url[rtsp_url.index("@"):]

        hoststr = hoststr[:hoststr.index("/"):]
        if hoststr.startswith("@"):
            hoststr = hoststr.replace("@", "")

        host_array = hoststr.split(":")

        user = cred_array[0]
        password = cred_array[1]
        host = host_array[0]
        port = int(host_array[1])

        fall_back_ports = [80, 443, 554]
        onvif_connection_test_result = test_onvif_connection(host, port, user, password)
        if onvif_connection_test_result == False:
            for fall_back_port in fall_back_ports:
                onvif_connection_test_result = test_onvif_connection(host, fall_back_port, user, password)
                if onvif_connection_test_result:
                    port = fall_back_port
                    break

        return (onvif_connection_test_result, user, password, host, port)
    except Exception as e:
        return (False, "", "", "", 0)          

def test_onvif_connection(rtsp_host, rtsp_port, rtsp_user, rtsp_password):
    try:
        # Connect to the camera
        mycam = ONVIFCamera(rtsp_host, rtsp_port, rtsp_user, rtsp_password, 'src/bob_monitor/resource/wsdl')
        return True
    except Exception as e:
        return False
