import traceback as tb
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from typing import List
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from bob_shared.node_runner import NodeRunner
from bob_interfaces.msg import TrackingState

class TrackerMonitorNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('bob_tracker_monitor')

    self.declare_parameters(
      namespace='',
      parameters=
      [('observer_tracker_monitor_timer_interval', 5),
       ('observer_tracking_profile_high_switch_threshold', 15)])

    self.timer_interval = self.get_parameter('observer_tracker_monitor_timer_interval').value
    self.tracking_profile_high_switch_threshold = self.get_parameter('observer_tracking_profile_high_switch_threshold').value

    self.msg_tracking_state = None

    self.timer = self.create_timer(self.timer_interval, self.tracking_monitor)

    #self.pub_environment_data = self.create_publisher(ObserverDayNight, 'bob/observer/day_night_classifier', publisher_qos_profile)

    # setup services, publishers and subscribers    
    self.sub_tracking_state = self.create_subscription(TrackingState, 'bob/tracker/tracking_state', self.tracking_state_callback, subscriber_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def tracking_state_callback(self, msg_tracking_state:TrackingState):
    self.msg_tracking_state = msg_tracking_state

  def tracking_monitor(self):

    if self.msg_tracking_state != None:

      try:

        if self.msg_tracking_state.alive > self.tracking_profile_high_switch_threshold:
          self.get_logger().warn(f'{self.get_name()} Tracker needs tuning to a lower sensitivity level as its breaching the threshold of: {self.tracking_profile_high_switch_threshold}')

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

  node = TrackerMonitorNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()