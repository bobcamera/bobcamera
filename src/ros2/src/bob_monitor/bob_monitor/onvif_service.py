import rclpy
from rclpy.node import Node
from bob_interfaces.srv import CameraSettings
import inspect
from onvif2 import ONVIFCamera

from ament_index_python.packages import get_package_share_directory


class ONVIFService(Node):
    def __init__(self):
        super().__init__('onvif_service')
        self.srv = self.create_service(CameraSettings, 'camera_settings', self.camera_settings_callback)
        self.log_info("ONVIF Service Initialized")

    def camera_settings_callback(self, request, response):
        # self.log_info(f"Received request: IP={request.ip}, Port={request.port}, User={request.user}")
        try:
            package_share_directory = get_package_share_directory('bob_monitor')
            wsdl_path = package_share_directory
            
            mycam = ONVIFCamera(request.host, request.port, request.user, request.password, wsdl_path)
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

            # self.log_info(f"Camera settings retrieved: Manufacturer={response.manufacturer}")
        except Exception as e:
            self.log_error(f"Error retrieving camera settings: {e}")

        return response

    def log_info(self, message):
        caller = inspect.stack()[1].function
        self.get_logger().info(f"[{__file__}:{caller}] {message}")

    def log_error(self, message):
        caller = inspect.stack()[1].function
        self.get_logger().error(f"[{__file__}:{caller}] {message}")

def main(args=None):
    rclpy.init(args=args)
    onvif_service = ONVIFService()
    rclpy.spin(onvif_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
