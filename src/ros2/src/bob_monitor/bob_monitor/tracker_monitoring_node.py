import traceback as tb
import rclpy
import numpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from bob_shared.node_runner import NodeRunner
from bob_interfaces.msg import TrackingState

class TrackerMonitorNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('bob_tracker_monitor')

    self.declare_parameters(
      namespace='',
      parameters=
      [('observer_tracker_monitor_busy_interval', 5),
       ('observer_tracker_monitor_idle_interval', 60),
       ('observer_tracking_profile_busy_switch_threshold', 5),
       ('observer_tracker_sample_set', 5)])

    self.timer_busy_interval = self.get_parameter('observer_tracker_monitor_busy_interval').value
    self.timer_idle_interval = self.get_parameter('observer_tracker_monitor_idle_interval').value
    self.tracking_profile_high_switch_threshold = self.get_parameter('observer_tracking_profile_busy_switch_threshold').value
    self.sample_set = self.get_parameter('observer_tracker_sample_set').value

    self.msg_tracking_state = None
    
    self.avg_tracks = [0]*self.sample_set
    self.track_counter = 0
    self.started = 0
    self.idle_enabled = False

    self.busy_timer = self.create_timer(self.timer_busy_interval, self.tracking_busy_monitor)
    self.idle_timer = self.create_timer(self.timer_idle_interval, self.tracking_idle_monitor)

    #self.pub_environment_data = self.create_publisher(ObserverDayNight, 'bob/observer/day_night_classifier', publisher_qos_profile)

    # setup services, publishers and subscribers    
    self.sub_tracking_state = self.create_subscription(TrackingState, 'bob/tracker/tracking_state', self.tracking_state_callback, subscriber_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def tracking_state_callback(self, msg_tracking_state:TrackingState):
    self.msg_tracking_state = msg_tracking_state

  def tracking_busy_monitor(self):

    if self.msg_tracking_state != None:

      try:

        # what is the delta tracks for the last second
        delta = int(self.msg_tracking_state.started) - self.started
        self.started = int(self.msg_tracking_state.started)

        self.avg_tracks[self.track_counter] = delta
        self.track_counter += 1
        if self.track_counter >= self.sample_set:
          self.track_counter = 0

        ma = numpy.sum(self.avg_tracks, dtype=numpy.int32) / self.sample_set

        #self.get_logger().info(f'{self.get_name()} BUSY ma: {ma}')

        if ma > self.tracking_profile_high_switch_threshold:
          self.get_logger().warn(f'{self.get_name()} Tracker needs tuning to a LOWER sensitivity level as its breaching the threshold of: {self.tracking_profile_high_switch_threshold} - Moving Average: {ma}')
          self.avg_tracks = [0]*self.sample_set # Reset the avg_tracks variable otherwise we might get several BUSY notifications

      except Exception as e:
        self.get_logger().error(f"Exception during alive track monitor. Error: {e}.")
        self.get_logger().error(tb.format_exc())
  
  def tracking_idle_monitor(self):

    if self.msg_tracking_state != None:

      try:

        # what is the delta tracks for the last second
        delta = int(self.msg_tracking_state.started) - self.started
        self.started = int(self.msg_tracking_state.started)

        self.avg_tracks[self.track_counter] = delta
        self.track_counter += 1
        if self.track_counter >= self.sample_set:
          self.track_counter = 0
          self.idle_enabled = True

        ma = numpy.sum(self.avg_tracks, dtype=numpy.int32) / self.sample_set

        #self.get_logger().info(f'{self.get_name()} IDLE ma: {ma}')

        if self.idle_enabled and ma == 0:
          self.get_logger().warn(f'{self.get_name()} Tracker should be tuned to a HIGHER sensitivity level if possible - Moving Average: {ma}')

      except Exception as e:
        self.get_logger().error(f"Exception during alive track monitor. Error: {e}.")
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