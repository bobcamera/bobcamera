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
    os.makedirs('assets/recordings/allsky', exist_ok=True)
    os.makedirs('assets/recordings/foreground_mask', exist_ok=True)
    os.makedirs('assets/recordings/heatmaps', exist_ok=True)
    os.makedirs('assets/recordings/json', exist_ok=True)
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

    simulation_height = int(LaunchConfiguration('simulation_height_arg').perform(context))
    simulation_width = int(LaunchConfiguration('simulation_width_arg').perform(context))
    simulation_num_objects = int(LaunchConfiguration('simulation_num_objects_arg').perform(context))

    bgs_algo = str(LaunchConfiguration('bgs_algorithm_arg').perform(context))

    tracking_mask_file = str(LaunchConfiguration('tracking_maskfile_arg').perform(context))
    tracking_use_mask = LaunchConfiguration('tracking_usemask_arg').perform(context) in ('True', 'true')
    tracking_mask_dir = os.path.dirname(tracking_mask_file)

    is_rtsp_source = source in ('\'rtsp\'', '\'rtsp_overlay\'')

    rtsp_user = rtsp_password = rtsp_host = rtsp_port = ''
    if is_rtsp_source:

        credstr = rtsp_url[rtsp_url.index("//"):rtsp_url.index("@")]
        if credstr.startswith("//"):
            credstr = credstr.replace("//", "")

        cred_array = credstr.split(":")

        hoststr = rtsp_url[rtsp_url.index("@"):]

        hoststr = hoststr[:hoststr.index("/"):]
        if hoststr.startswith("@"):
            hoststr = hoststr.replace("@", "")

        host_array = hoststr.split(":")

        rtsp_user = cred_array[0]
        rtsp_password = cred_array[1]
        rtsp_host = host_array[0]
        rtsp_port = int(host_array[1])

    update_config = LaunchConfiguration('update_config_from_env_vars_arg').perform(context) in ('True', 'true')

    if update_config:
        
        with open(app_config_file_src, 'r') as read:
            yaml_output = yaml.safe_load(read)

            # rstp_camera_node
            yaml_output['rstp_camera_node']['ros__parameters']['rtsp_uri'] = rtsp_url
            yaml_output['rstp_camera_node']['ros__parameters']['image_width'] = image_width
            yaml_output['rstp_camera_node']['ros__parameters']['image_height'] = image_height
        
            # rstp_overlay_camera_node
            yaml_output['rstp_overlay_camera_node']['ros__parameters']['rtsp_uri'] = rtsp_url
            yaml_output['rstp_overlay_camera_node']['ros__parameters']['image_width'] = image_width
            yaml_output['rstp_overlay_camera_node']['ros__parameters']['image_height'] = image_height

            # web_camera_video_node
            yaml_output['usb_camera_node']['ros__parameters']['camera_id'] = camera_id

            # simulated_frame_provider_node
            yaml_output['simulated_frame_provider_node']['ros__parameters']['num_objects'] = simulation_num_objects
            yaml_output['simulated_frame_provider_node']['ros__parameters']['height'] = simulation_height
            yaml_output['simulated_frame_provider_node']['ros__parameters']['width'] = simulation_width

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

            if is_rtsp_source:
                yaml_output['rstp_camera_node']['ros__parameters']['onvif_user'] = rtsp_user
                yaml_output['rstp_camera_node']['ros__parameters']['onvif_password'] = rtsp_password
                yaml_output['rstp_camera_node']['ros__parameters']['onvif_host'] = rtsp_host
                yaml_output['rstp_camera_node']['ros__parameters']['onvif_port'] = rtsp_port

                # TODO: Can't get this to work for the moment
                # get_camera_settings(yaml_output, rtsp_host, rtsp_port, rtsp_user, rtsp_password, 'src/bob_monitor/resource/wsdl')

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
        create_storage_folders_func,
        application_config_func,
        camera_info_config_func,
    ])

def createImagingRequest(imaging, token):
	requestSetImagingSettings = imaging.create_type("SetImagingSettings")
	requestSetImagingSettings.VideoSourceToken = token
	requestSetImagingSettings.ImagingSettings = imaging.GetImagingSettings({'VideoSourceToken': token})
	return requestSetImagingSettings

def get_camera_settings(yaml_output, ip, port, user, password, wdsl_dir):
    try:
        # Connect to the camera
        mycam = ONVIFCamera(ip, port, user, password, wdsl_dir)

        device_info = mycam.devicemgmt.GetDeviceInformation()

        print("\nCamera Information:")
        print(f"Manufacturer: {device_info.Manufacturer}")
        print(f"Model: {device_info.Model}")
        print(f"Firmware Version: {device_info.FirmwareVersion}")
        print(f"Serial Number: {device_info.SerialNumber}")
        print(f"Hardware ID: {device_info.HardwareId}")

        yaml_output['onvif_service_node']['ros__parameters']['manufacturer'] = device_info.Manufacturer
        yaml_output['onvif_service_node']['ros__parameters']['model'] = device_info.Model
        yaml_output['onvif_service_node']['ros__parameters']['firmware_version'] = device_info.FirmwareVersion
        yaml_output['onvif_service_node']['ros__parameters']['serial_number'] = device_info.SerialNumber
        yaml_output['onvif_service_node']['ros__parameters']['hardware_id'] = device_info.HardwareId

        media2_service = mycam.create_media2_service()
        profiles = media2_service.GetProfiles()
    
        print("\nMedia Profiles and Stream URIs:")
        for profile in profiles:
            o = media2_service.create_type('GetStreamUri')
            o.ProfileToken = profile.token
            o.Protocol = 'RTSP'
            uri = media2_service.GetStreamUri(o)  
            print(f"Token: {profile.token}, RTSP URI: {uri}")

        print("\nVideo Encoder Configurations:")
        configurations = media2_service.GetVideoEncoderConfigurations()
        for configuration in configurations:
            encoding = configuration['Encoding'].lower()
            if encoding in ['h264', 'h265']:
                width = configuration['Resolution']['Width']
                height = configuration['Resolution']['Height']
                fps = configuration['RateControl']['FrameRateLimit']
                bitrate = configuration['RateControl']['BitrateLimit']
                cbr = configuration['RateControl']['ConstantBitRate']
                gop = configuration['GovLength']
                profile = configuration['Profile']
                quality = configuration['Quality']

                print(f"Token: {configuration['token']}, Encoding: {configuration['Encoding']}, "
                    f"Resolution: {width}x{height}, FPS: {fps}, Bitrate: {bitrate}, CBR: {cbr}, "
                    f"GOP: {gop}, Profile: {profile}, Quality: {quality}")
            else:
                print(f"Token: {configuration['token']}, Encoding: {configuration['Encoding']}")

        # Doesn't work for media2 currently / h265
        media_service = mycam.create_media_service()
        media_profile = media_service.GetProfiles()[0]
        imaging = mycam.create_imaging_service()
        token = media_profile.VideoSourceConfiguration.SourceToken
        request = createImagingRequest(imaging, token)
        imaging_settings = request.ImagingSettings
        
        print("\nImaging Settings:")
        print(f"Brightness: {imaging_settings.Brightness}")
        print(f"Contrast: {imaging_settings.Contrast}")
        print(f"Saturation: {imaging_settings.ColorSaturation}")
        print(f"Sharpness: {imaging_settings.Sharpness}")
        # Add other settings as required
        
    except Exception as e:
        print(f"An error occurred: {e}")

