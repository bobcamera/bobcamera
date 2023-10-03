import traceback as tb
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import List
from sensor_msgs.msg import Image
from bob_shared.node_runner import NodeRunner
from .object_simulator import MovingCircle

class SimulationOverlayProviderNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('bob_overlayed_video_provider')

    self.br = CvBridge()

    self.declare_parameters(
        namespace="",
        parameters=[                
            ("height", 1080),
            ("width", 1920),
            ("target_object_diameter", 5)])

    # grab parameters provided
    self.height = self.get_parameter('height').value
    self.width = self.get_parameter('width').value
    self.target_object_diameter = self.get_parameter('target_object_diameter').value

    self.moving_circles = [MovingCircle(self.width, self.height, 0) for i in range(4)]

    # setup services, publishers and subscribers
    self.sub_camera = self.create_subscription(Image, 'bob/simulation/input_frame', self.camera_callback, subscriber_qos_profile)
    self.pub_synthetic_frame = self.create_publisher(Image, 'bob/simulation/output_frame', publisher_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def camera_callback(self, msg_image:Image):

    if not msg_image is None:

      try:
        input_frame = self.br.imgmsg_to_cv2(msg_image)
        for circle in self.moving_circles:
            circle.move()
            circle.draw(input_frame, (10, 10, 20))
        frame_synthetic_msg = self.br.cv2_to_imgmsg(input_frame, encoding=msg_image.encoding)
        frame_synthetic_msg.header = msg_image.header
        self.pub_synthetic_frame.publish(frame_synthetic_msg)        
      except Exception as e:
        self.get_logger().error(f"Exception during frame overlay simulation. Error: {e}.")
        self.get_logger().error(tb.format_exc())

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

  node = SimulationOverlayProviderNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()
