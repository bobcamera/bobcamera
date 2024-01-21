import rclpy
from rclpy.node import Node
from bob_interfaces.srv import CameraSettings
import inspect
from onvif2 import ONVIFCamera

from bob_shared.node_runner import NodeRunner
# onvif-cli 
#  /bin/python3 /home/ros/.local/lib/python3.10/site-packages/onvif2/cli.py -u 'bob' -a 'Sky360Sky!' --host '10.20.30.140' --port 80 --wsdl /workspaces/bobcamera/src/ros2/src/bob_monitor/resource/wsdl
# >>> cmd devicemgmt GetCapabilities {'Category': 'PTZ'}
# True: OrderedDict([('Analytics', None), ('Device', None), ('Events', None), ('Imaging', None), ('Media', None), 
# ('PTZ', OrderedDict([('XAddr', 'http://10.20.30.140/onvif/PTZ'), ('_value_1', None), ('_attr_1', None)])), 
#('Extension', OrderedDict([('_value_1', [<Element {http://www.onvifext.com/onvif/ext/ver10/schema}hikCapabilities at 0x7fe5db816fc0>]), ('DeviceIO', None), ('Display', None), ('Recording', None), ('Search', None), ('Replay', None), ('Receiver', None), ('AnalyticsDevice', None), ('Extensions', None)])), ('_attr_1', None)])
# cmd  ptz GotoHomePosition {'ProfileToken': 'Profile_3'}
# cmd ptz Stop {'ProfileToken': 'Profile_1'}
# cmd ptz ContinuousMove {'ProfileToken': 'Profile_1','Velocity': {'PanTilt': {'x': 0,'y': 0.2},'Zoom': {'x': 0.0}}}
# Continous Move Velocity: x: -1...+1, y: -1...+1
# cmd ptz AbsoluteMove {'ProfileToken': 'Profile_1','Position': {'PanTilt': {'x': 0,'y': 0.0},'Zoom': {'x': 0.0}}}
# cmd ptz AbsoluteMove {'ProfileToken': 'Profile_1','Position': {'PanTilt': {'x': -0.55,'y': -0.5001},'Zoom': {'x': 0.0}}}
# Absolute position: x: -1...+1, y: -1...+1
# cmd ptz RelativeMove {'ProfileToken': 'Profile_1','Translation': {'PanTilt': {'x': 0.2,'y': 0},'Zoom': {'x': 0.0}}}





class ONVIFServiceNode(Node):
    def __init__(self):
        super().__init__('onvif_service')

        self.declare_parameters(namespace='', parameters=[('onvif_wsdl_path', 'src/bob_monitor/resource/wsdl')])
        self.wsdl_path = self.get_parameter('onvif_wsdl_path').value
        self.srv = self.create_service(CameraSettings, 'camera_settings', self.camera_settings_callback)
        self.log_info("ONVIF Service Initialized")

    def camera_settings_callback(self, request, response):
        # self.log_info(f"Received request: IP={request.ip}, Port={request.port}, User={request.user}")
        try:
            mycam = ONVIFCamera(request.host, request.port, request.user, request.password, self.wsdl_path)
            # Populate the response with the necessary information
            device_info = mycam.devicemgmt.GetDeviceInformation()
            response.manufacturer = device_info.Manufacturer
            response.model = device_info.Model
            response.firmware_version = device_info.FirmwareVersion
            response.serial_number = device_info.SerialNumber
            response.hardware_id = device_info.HardwareId
            
            media2_service = mycam.create_media2_service()
            configurations = media2_service.GetVideoEncoderConfigurations()
            
            response.num_configurations = len(configurations)
            response.encoding = configurations[0]['Encoding'].lower()

            response.success = True

            self.get_logger().info(f"Successfully retrieved your {device_info.Manufacturer} camera settings using ONVIF.")
            #self.log_camera_details(mycam)
        except Exception as e:
            self.get_logger().error(f"Error retrieving camera settings using ONVIF: {e}")
            response.success = False

        return response

    def log_info(self, message):
        caller = inspect.stack()[1].function
        self.get_logger().info(f"[{__file__}:{caller}] {message}")

    def log_error(self, message):
        caller = inspect.stack()[1].function
        self.get_logger().error(f"[{__file__}:{caller}] {message}")

    def log_camera_details(self, mycam):
        
        try:
            device_info = mycam.devicemgmt.GetDeviceInformation()
            self.get_logger().info("\nCamera Information:")
            self.get_logger().info(f"Manufacturer: {device_info.Manufacturer}")
            self.get_logger().info(f"Model: {device_info.Model}")
            self.get_logger().info(f"Firmware Version: {device_info.FirmwareVersion}")
            self.get_logger().info(f"Serial Number: {device_info.SerialNumber}")
            self.get_logger().info(f"Hardware ID: {device_info.HardwareId}")

            media2_service = mycam.create_media2_service()
            profiles = media2_service.GetProfiles()
        
            self.get_logger().info("\nMedia Profiles and Stream URIs:")
            for profile in profiles:
                o = media2_service.create_type('GetStreamUri')
                o.ProfileToken = profile.token
                o.Protocol = 'RTSP'
                uri = media2_service.GetStreamUri(o)  
                self.get_logger().info(f"Token: {profile.token}, RTSP URI: {uri}")

            self.get_logger().info("\nVideo Encoder Configurations:")
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

                    self.get_logger().info(f"Token: {configuration['token']}, Encoding: {configuration['Encoding']}, "
                        f"Resolution: {width}x{height}, FPS: {fps}, Bitrate: {bitrate}, CBR: {cbr}, "
                        f"GOP: {gop}, Profile: {profile}, Quality: {quality}")
                else:
                    self.get_logger().info(f"Token: {configuration['token']}, Encoding: {configuration['Encoding']}")

            # Doesn't work for media2 currently / h265
            media_service = mycam.create_media_service()
            #print("First try RelativeMove")
            #mycam.ptz.RelativeMove({'ProfileToken': 'Profile_1','Translation': {'PanTilt': {'x': 0.2,'y': 0},'Zoom': {'x': 0.0}}})
            #print("Second try RelativeMove")
            #mycam.ptz.RelativeMove("{'ProfileToken': 'Profile_1','Translation': {'PanTilt': {'x': 0.2,'y': 0},'Zoom': {'x': 0.0}}}")
            media_profile = media_service.GetProfiles()[0]
            imaging = mycam.create_imaging_service()
            token = media_profile.VideoSourceConfiguration.SourceToken
            request = self.createImagingRequest(imaging, token)
            imaging_settings = request.ImagingSettings
            
            self.get_logger().info("\nImaging Settings:")
            self.get_logger().info(f"Brightness: {imaging_settings.Brightness}")
            self.get_logger().info(f"Contrast: {imaging_settings.Contrast}")
            self.get_logger().info(f"Saturation: {imaging_settings.ColorSaturation}")
            self.get_logger().info(f"Sharpness: {imaging_settings.Sharpness}")

        except Exception as e:
            self.log_error(f"Error retrieving detailed camera settings: {e}")        

    def createImagingRequest(self, imaging, token):
        requestSetImagingSettings = imaging.create_type("SetImagingSettings")
        requestSetImagingSettings.VideoSourceToken = token
        requestSetImagingSettings.ImagingSettings = imaging.GetImagingSettings({'VideoSourceToken': token})
        return requestSetImagingSettings

def main(args=None):
    rclpy.init(args=args)

    node = ONVIFServiceNode()

    runner = NodeRunner(node)
    runner.run()

if __name__ == '__main__':
    main()
