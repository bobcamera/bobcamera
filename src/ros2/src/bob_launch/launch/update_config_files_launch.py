import os
import yaml
import cv2
import shutil
from onvif2 import ONVIFCamera
from launch.actions import LogInfo
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration 
from ament_index_python.packages import get_package_share_directory

class ConfigDiscoverer():

    def __init__(self, source, rtsp_url, camera_id, videos):

        self.source = source
        self.rtsp_url = rtsp_url
        self.camera_id = camera_id
        self.videos = videos

        self.rtsp_user = ""
        self.rtsp_password = ""
        self.rtsp_host = ""
        self.rtsp_port = 0

        self.height = 0
        self.width = 0
        self.fps = 0
        #self.bitrate = 10240000

        self.onvif_success = False

        self.onvif_profile_for_settings_determination = 'main'
        self.wsdl_location = 'assets/wsdl'
        self.resolutions = {3840:2160, 2560:1440, 1920:1080, 1600:900, 1280:720, 1024:768, 800:600, 640:480, 320:240}

    def discover(self):

        success = False
        video_capture = None

        try:

            if self.source in ('\'simulate\''):
                print(f"The simulate option has been selected, its using hard coded values")
                self.width = 2880
                self.height = 1620
                self.fps = 25.0
                success = True

            if self.source in ('\'rtsp\'', '\'rtsp_overlay\''):
                (onvif_success_, rtsp_user_, rtsp_password_, rtsp_host_, rtsp_port_) = self._get_onvif_config(self.rtsp_url)
                if onvif_success_:
                    self.onvif_success = onvif_success_
                    self.rtsp_user = rtsp_user_
                    self.rtsp_password = rtsp_password_
                    self.rtsp_host = rtsp_host_
                    self.rtsp_port = rtsp_port_

                    (retrieval_success, width_, height_, fps_, bitrate_) = self._get_image_stream_details_using_onvif(rtsp_user_, rtsp_password_, rtsp_host_, rtsp_port_)
                    if retrieval_success:
                        self.width = width_
                        self.height = height_
                        self.fps = fps_
                        self.bitrate = bitrate_
                        success = True
                    else:
                        video_capture = cv2.VideoCapture(self.rtsp_url)
                else:
                    video_capture = cv2.VideoCapture(self.rtsp_url)

            if self.source in ('\'usb\''):

                video_capture = cv2.VideoCapture(self.camera_id)

                for key in self.resolutions:
                    w = key
                    h = self.resolutions[key]
                    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                    if (video_capture.get(cv2.CAP_PROP_FRAME_WIDTH) == w and video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT) == h):
                        break

            if self.source in ('\'video\'', '\'video_overlay\''):
                videos_ = self.videos.split(";")
                video_capture = cv2.VideoCapture(videos_[0])

            if video_capture is not None:
                self.fps = video_capture.get(cv2.CAP_PROP_FPS)

                counter = 0
                while True:
                    success, frame = video_capture.read()
                    counter = counter + 1
                    if success:
                        self.height, self.width, channels = frame.shape
                        print(f"Discovery - OpenCV (v{cv2.__version__}) Details :- width:{self.width}, height: {self.height} @ FPS: {self.fps}")
                        success = True
                        break
                    
                    if counter > 10:
                        break

        except Exception as e:
            print(f"Error discovering Video settings: {e}")

        if success:
            print(f"Video settings discovery: SUCCESS")
        else:
            print(f"Video settings discovery: FAIL")

        return success

    def _get_image_stream_details_using_onvif(self, rtsp_user, rtsp_password, rtsp_host, rtsp_port):

        #print(f"Using ONVIF to search the Main RTSP profile for image details...")

        try:
            mycam = ONVIFCamera(rtsp_host, rtsp_port, rtsp_user, rtsp_password, self.wsdl_location)
            media2_service = mycam.create_media2_service()
    
            configurations = media2_service.GetVideoEncoderConfigurations()
            for configuration in configurations:
                encoding = configuration['Encoding'].lower()
                if encoding in ['h264', 'h265']:

                    width = configuration['Resolution']['Width']
                    height = configuration['Resolution']['Height']
                    fps = configuration['RateControl']['FrameRateLimit']
                    bitrate = configuration['RateControl']['BitrateLimit']
                    profile = configuration['Profile']

                    if profile.lower() in [self.onvif_profile_for_settings_determination]:

                        print(f"Discovery - ONVIF ({self.onvif_profile_for_settings_determination}) Details :- width:{width}, height: {height} @ FPS: {fps}, @ Bitrate: {bitrate}")
                        return (True, width, height, fps, bitrate)

        except Exception as e:
            print(f"Error retrieving camera settings using ONVIF: {e}")
        
        return (False, 0, 0, 0, 0)

    def _get_onvif_config(self, rtsp_url):

        try:
            credstr = rtsp_url[rtsp_url.index("//"):rtsp_url.rfind("@")]
            if credstr.startswith("//"):
                credstr = credstr.replace("//", "")

            cred_array = credstr.split(":")

            hoststr = rtsp_url[rtsp_url.rfind("@"):]

            hoststr = hoststr[:hoststr.index("/"):]
            if hoststr.startswith("@"):
                hoststr = hoststr.replace("@", "")

            host_array = hoststr.split(":")

            user = cred_array[0]
            password = cred_array[1]
            host = host_array[0]
            port = int(host_array[1])

            # print(f"User: {user}, Password: {password}, Host: {host}, Port: {port}")

            fall_back_ports = [80, 443, 554]
            onvif_connection_test_result = self._test_onvif_connection(host, port, user, password)
            if onvif_connection_test_result == False:
                for fall_back_port in fall_back_ports:
                    onvif_connection_test_result = self._test_onvif_connection(host, fall_back_port, user, password)
                    if onvif_connection_test_result:
                        port = fall_back_port
                        break

            return (onvif_connection_test_result, user, password, host, port)
        except Exception as e:
            return (False, "", "", "", 0)    

    def _test_onvif_connection(self, rtsp_host, rtsp_port, rtsp_user, rtsp_password):
        try:
            # Connect to the camera
            mycam = ONVIFCamera(rtsp_host, rtsp_port, rtsp_user, rtsp_password, self.wsdl_location)
            return True
        except Exception as e:
            #print(f"Error connecting to RTSP camera using ONVIF ({rtsp_user}:{rtsp_password}@{rtsp_host}:{rtsp_port}): {e}")
            return False

def create_storage_folders(context):
    # TODO: Parameterise these a bit better, or drive them from the UI or something but for now
    # hard coding them will have to do.
    os.makedirs('assets', exist_ok=True)
    os.makedirs('assets/config', exist_ok=True)
    os.makedirs('assets/recordings', exist_ok=True)
    os.makedirs('assets/masks', exist_ok=True)
    os.makedirs('assets/wsdl', exist_ok=True)
    if not os.path.isdir('assets/wsdl/ver10'):
        shutil.copytree('install/bob_monitor/share/bob_monitor/ver10', 'assets/wsdl/ver10')
    if not os.path.isdir('assets/wsdl/ver20'):
        shutil.copytree('install/bob_monitor/share/bob_monitor/ver20', 'assets/wsdl/ver20')

def application_config(context):

    app_config_file_src = os.path.join(get_package_share_directory('bob_launch'), 'config', 'app_config.yaml')
    app_config_file = 'assets/config/app_config.yaml'

    # get the values provided as part of the launch arguments
    source = str(LaunchConfiguration('source_arg').perform(context))
    rtsp_url = str(LaunchConfiguration('rtsp_url_arg').perform(context))
    camera_id = int(LaunchConfiguration('camera_id_arg').perform(context))
    videos = str(LaunchConfiguration('video_arg').perform(context))
    
    enable_visualiser = LaunchConfiguration('enable_visualiser_arg').perform(context) in ('True', 'true')

    image_width = int(LaunchConfiguration('rtsp_width_arg').perform(context))
    image_height = int(LaunchConfiguration('rtsp_height_arg').perform(context))
    fps = float(LaunchConfiguration('fps_arg').perform(context))

    simulation_height = int(LaunchConfiguration('simulation_height_arg').perform(context))
    simulation_width = int(LaunchConfiguration('simulation_width_arg').perform(context))
    simulation_num_objects = int(LaunchConfiguration('simulation_num_objects_arg').perform(context))

    bgs_algo = str(LaunchConfiguration('bgs_algorithm_arg').perform(context))
    tracking_sensitivity = str(LaunchConfiguration('tracking_sensitivity_arg').perform(context)).replace("'", "")

    enable_star_mask = LaunchConfiguration('enable_star_mask_arg').perform(context) in ('True', 'true')

    tracking_mask_dir = 'assets/masks'

    #bitrate_str = 'bitrate=10240000'
    mask_enable_override = source in ('\'rtsp\'', '\'rtsp_overlay\'', '\'usb\'')
    mask_enable_offset_correction = source in ('\'rtsp\'', '\'rtsp_overlay\'', '\'usb\'')

    update_config = LaunchConfiguration('update_config_from_env_vars_arg').perform(context) in ('True', 'true')    

    discoverer = ConfigDiscoverer(source, rtsp_url, camera_id, videos)
    if discoverer.discover():
        image_width = discoverer.width
        image_height = discoverer.height
        simulation_width = discoverer.width
        simulation_height = discoverer.height        
        fps = discoverer.fps
        #bitrate_str = f'bitrate={discoverer.bitrate}'

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

            # detection_mask_application_node
            yaml_output['detection_mask_application_node']['ros__parameters']['mask_enable_override'] = mask_enable_override
            yaml_output['detection_mask_application_node']['ros__parameters']['mask_enable_offset_correction'] = mask_enable_offset_correction
            yaml_output['detection_mask_application_node']['ros__parameters']['image_width'] = image_width
            yaml_output['detection_mask_application_node']['ros__parameters']['image_height'] = image_height

            # privacy_mask_application_node
            yaml_output['privacy_mask_application_node']['ros__parameters']['mask_enable_override'] = mask_enable_override
            yaml_output['privacy_mask_application_node']['ros__parameters']['image_width'] = image_width
            yaml_output['privacy_mask_application_node']['ros__parameters']['image_height'] = image_height            

            # background_subtractor_node
            yaml_output['background_subtractor_node']['ros__parameters']['bgs'] = bgs_algo
            yaml_output['background_subtractor_node']['ros__parameters']['sensitivity'] = tracking_sensitivity

            # track_sensitivity_monitor_node
            yaml_output['track_sensitivity_monitor_node']['ros__parameters']['sensitivity'] = tracking_sensitivity
            yaml_output['track_sensitivity_monitor_node']['ros__parameters']['star_mask_enabled'] = enable_star_mask

            # Tracking status message on annotated frame
            yaml_output['annotated_frame_provider_node']['ros__parameters']['tracking_status_message'] = enable_visualiser

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
            #yaml_output['allsky_recorder_node']['ros__parameters']['pipeline'] = f'appsrc ! videoconvert ! openh264enc {bitrate_str} qp-min=10 qp-max=51 ! h264parse ! mp4mux ! filesink location='

            # onvif details
            if discoverer.onvif_success:
                yaml_output['rtsp_camera_node']['ros__parameters']['onvif_user'] = discoverer.rtsp_user
                yaml_output['rtsp_camera_node']['ros__parameters']['onvif_password'] = discoverer.rtsp_password
                yaml_output['rtsp_camera_node']['ros__parameters']['onvif_host'] = discoverer.rtsp_host
                yaml_output['rtsp_camera_node']['ros__parameters']['onvif_port'] = discoverer.rtsp_port

                yaml_output['rtsp_overlay_camera_node']['ros__parameters']['onvif_user'] = discoverer.rtsp_user
                yaml_output['rtsp_overlay_camera_node']['ros__parameters']['onvif_password'] = discoverer.rtsp_password
                yaml_output['rtsp_overlay_camera_node']['ros__parameters']['onvif_host'] = discoverer.rtsp_host
                yaml_output['rtsp_overlay_camera_node']['ros__parameters']['onvif_port'] = discoverer.rtsp_port

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
    #camera_info_config_func = OpaqueFunction(function = camera_config)

    return LaunchDescription([
        LogInfo(msg=['Updating config files, this might take a minute, please wait...']),
        create_storage_folders_func,
        application_config_func,
        #camera_info_config_func,
        LogInfo(msg=['Config files update complete.']),
    ])
