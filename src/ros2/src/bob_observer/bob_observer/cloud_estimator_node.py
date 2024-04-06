import traceback as tb
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from typing import List
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from bob_shared.enumerations import DayNightEnum
from bob_shared.node_runner import NodeRunner
from bob_interfaces.msg import ObserverCloudEstimation, ObserverDayNight
from .cloud_estimator import CloudEstimator

class CloudEstimatorNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('bob_cloud_estimator')

    self.declare_parameters(namespace='',
                            parameters=[('observer_timer_interval', 30)])

    self.br = CvBridge()
    self.timer_interval = self.get_parameter('observer_timer_interval').value
    self.msg_image = None
    self.day_night: DayNightEnum = DayNightEnum.Unknown
    self.day_cloud_estimator = CloudEstimator.Day()
    self.night_cloud_estimator = CloudEstimator.Night()

    self.timer = self.create_timer(self.timer_interval, self.cloud_sampler)

    self.pub_environment_data = self.create_publisher(ObserverCloudEstimation, 'bob/observer/cloud_estimation', publisher_qos_profile)

    # setup services, publishers and subscribers    
    self.sub_camera = self.create_subscription(Image, 'bob/observer_frame/source', self.camera_callback, subscriber_qos_profile)
    self.sub_environment_day_night = self.create_subscription(ObserverDayNight, 'bob/observer/day_night_classifier', 
      self.day_night_callback, subscriber_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_callback(self, msg_image:Image):
    self.msg_image = msg_image

  def day_night_callback(self, msg_day_night:ObserverDayNight):
    self.day_night = DayNightEnum(msg_day_night.day_night_enum)

  def cloud_sampler(self):
    
    if self.msg_image != None:

      estimation: float = None
      distribution: bool = None

      try:

        match self.day_night:
          case DayNightEnum.Day:
            estimation, distribution = self.day_cloud_estimator.estimate(self.br.imgmsg_to_cv2(self.msg_image))
            self.get_logger().info(f'{self.get_name()} Day time cloud estimation --> {estimation}')
          case DayNightEnum.Night:
            estimation, _ = self.night_cloud_estimator.estimate(self.br.imgmsg_to_cv2(self.msg_image))
            self.get_logger().info(f'{self.get_name()} Night time cloud estimation --> {estimation}')
          case _:
            self.get_logger().info(f'{self.get_name()} Unknown Day/Night classifier, ignore for now')
            pass
        
        if estimation != None:
          cloud_estimation_msg = ObserverCloudEstimation()
          cloud_estimation_msg.percentage_cloud_cover = estimation
          cloud_estimation_msg.unimodal_cloud_cover = distribution
          self.pub_environment_data.publish(cloud_estimation_msg)

      except Exception as e:
        self.get_logger().error(f"Exception during cloud estimation sampler. Error: {e}.")
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

  node = CloudEstimatorNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()