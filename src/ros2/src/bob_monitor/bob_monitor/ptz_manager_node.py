import traceback as tb
import rclpy
import numpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from bob_shared.node_runner import NodeRunner
from bob_interfaces.msg import Tracking
from .detection import DetectionCounter2

class PTZManagerNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('ptz_manager')

    self.declare_parameters(
      namespace='',
      parameters=[
        ('observer_ptz_manager_velocity_threshold', 5),
        ('observer_ptz_manager_fps', 15)        
        ])

    self.velocity_threshold = self.get_parameter('observer_ptz_manager_velocity_threshold').value
    self.observer_ptz_manager_fps = self.get_parameter('observer_ptz_manager_fps').value

    self.detection_counter = DetectionCounter2(self.get_logger())
    
    # setup services, publishers and subscribers    
    self.sub_tracking_state = self.create_subscription(Tracking, 'bob/tracker/tracking', self.tracking_state_callback, subscriber_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def tracking_state_callback(self, msg_tracking:Tracking):
    self.detection_counter.update(msg_tracking.detections)

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

  node = PTZManagerNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()