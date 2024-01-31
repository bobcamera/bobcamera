import cv2
import os
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from bob_interfaces.srv import ImageRaster
from bob_shared.node_runner import NodeRunner

class RasterImageAcquisitionService(Node):
    def __init__(self, service_qos_profile: QoSProfile):
        super().__init__('raster_image_acquisition_service')

        # Camera URIs
        self.fisheye_camera_uri = 'rtsp://jdm:bobcamera!@192.168.1.20:554/cam/realmonitor?channel=1&subtype=0'
        self.ptz_camera_uri = 'rtsp://jdm:bobcamera!@192.168.1.20:554/cam/realmonitor?channel=1&subtype=0'

        # Directory for saving calibration images
        self.base_image_dir = 'assets/calibration'

        # Reading frames from cameras
        self.fisheye_capture = cv2.VideoCapture(self.fisheye_camera_uri)
        self.ptz_capture = cv2.VideoCapture(self.ptz_camera_uri)

        # Create service for ImageAcquisition
        self.service = self.create_service(ImageRaster, '/image_acquisition', self.image_acquisition_callback)

    def image_acquisition_callback(self, request, response):
        x = request.x
        y = request.y
        zoom = request.zoom
        campaign = request.campaign

        self.current_campaign = campaign
        self.get_logger().info(f"Received image acquisition request: x={x}, y={y}, zoom={zoom}, campaign={campaign}")

        ret1, frame1 = self.fisheye_capture.read()
        # time.sleep(0.1)
        ret2, frame2 = self.ptz_capture.read()
        
        if ret1 and ret2:
            campaign_folder = os.path.join(self.base_image_dir, self.current_campaign)
            os.makedirs(campaign_folder, exist_ok=True)
            time_now = self.get_time_now_string()
            self.get_logger().info(f"Fisheye timestamp is: {self.fisheye_capture.get(cv2.CAP_PROP_POS_MSEC)}")
            self.get_logger().info(f"PTZ timestamp is: {self.ptz_capture.get(cv2.CAP_PROP_POS_MSEC)}")
            self.save_image(frame1, campaign_folder, 'ptz', x, y, zoom, time_now)
            self.save_image(frame2, campaign_folder, 'fisheye', x, y, zoom, time_now)

            # Set response success
            response.success = True
        else:
            response.success = False

        return response

    def save_image(self, frame, folder, prefix, x, y, zoom, time_now):
        filename = f"{prefix}_{x}_{y}_{zoom}_{time_now}.png"
        filepath = os.path.join(folder, filename)
        cv2.imwrite(filepath, frame)

    def get_time_now_string(self):
        now = self.get_clock().now()
        return str(now.to_msg().sec)  # Using seconds for simplicity

def main(args=None):
    rclpy.init(args=args)
    
    service_qos_profile = QoSProfile(depth=10)
    service_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
    service_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
    service_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
  
    node = RasterImageAcquisitionService(service_qos_profile)
    runner = NodeRunner(node)
    runner.run()

if __name__ == '__main__':
    main()
