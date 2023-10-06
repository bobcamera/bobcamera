import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from typing import List
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2
from bob_shared.node_runner import NodeRunner

class ImageToVideoConverter(Node): 
    
    def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
        super().__init__('bob_image_to_video_converter')
        self.bridge = CvBridge() 
        self.subscription = self.create_subscription( CompressedImage, 'bob/frames/annotated/resized/compressed', self.image_callback, subscriber_qos_profile)
        self.video_writer = None

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if self.video_writer is None:
                self.init_video_writer(cv_image)
            self.video_writer.write(cv_image)
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

    def init_video_writer(self, image):
        try:
            height, width, _ = image.shape
            video_format = 'mp4'  # or any other video format supported by OpenCV
            video_filename = 'recordings/videos/output_video.' + video_format
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 30  # Frames per second
            self.video_writer = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
        except Exception as e:
            self.get_logger().error('Error initializing video writer: %s' % str(e))

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()        
        super().destroy_node()

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = QoSProfile(depth=10)
  subscriber_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
  subscriber_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
  subscriber_qos_profile.history = QoSHistoryPolicy.KEEP_LAST

  publisher_qos_profile = QoSProfile(depth=10)
  publisher_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
  publisher_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
  publisher_qos_profile.history = QoSHistoryPolicy.KEEP_LAST

  node = ImageToVideoConverter(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()

if __name__ == '__main__':
  main()