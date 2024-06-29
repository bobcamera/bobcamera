import traceback as tb
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from typing import List
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from bob_shared.node_runner import NodeRunner
from bob_shared.enumerations import DayNightEnum
from bob_interfaces.msg import ObserverDayNight
from .day_night_classifier import DayNightEstimator

class DayNightClassifierNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('bob_day_night_estimator')

    self.declare_parameters(
      namespace='',
      parameters=
      [('observer_timer_interval', 30),
       ('observer_day_night_brightness_threshold', 95)])

    self.br = CvBridge()
    self.timer_interval = self.get_parameter('observer_timer_interval').value
    self.msg_image = None
    self.day_night_estimator = DayNightEstimator.Classifier(self.get_parameter('observer_day_night_brightness_threshold').value)

    self.timer = self.create_timer(self.timer_interval, self.day_night_classifier)

    self.pub_environment_data = self.create_publisher(ObserverDayNight, 'bob/observer/day_night_classifier', publisher_qos_profile)

    # setup services, publishers and subscribers    
    self.sub_camera = self.create_subscription(Image, 'bob/observer_frame/source', self.camera_callback, subscriber_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_callback(self, msg_image:Image):
    self.msg_image = msg_image

  def day_night_classifier(self):

    if self.msg_image != None:

      try:

        result, average_brightness = self.day_night_estimator.estimate(self.br.imgmsg_to_cv2(self.msg_image))
        self.get_logger().info(f'{self.get_name()} Day/Night classifier --> {str(result)}, {average_brightness}')

        day_night_msg = ObserverDayNight()
        day_night_msg.day_night_enum = int(result)
        day_night_msg.avg_brightness = average_brightness
        self.pub_environment_data.publish(day_night_msg)

      except Exception as e:
        self.get_logger().error(f"Exception during day night classification. Error: {e}.")
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

  node = DayNightClassifierNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()