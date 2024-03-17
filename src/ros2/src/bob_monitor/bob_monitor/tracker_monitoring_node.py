import traceback as tb
import rclpy
import numpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from bob_shared.node_runner import NodeRunner
from bob_interfaces.msg import TrackingState, RecordingState, DetectorState
from bob_interfaces.srv import BGSResetRequest

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
    self.msg_recording_state = None
    self.msg_detector_state = None
    
    self.avg_tracks = [0]*self.sample_set
    self.track_counter = 0
    self.started = 0
    self.idle_enabled = False

    self.busy_timer = self.create_timer(self.timer_busy_interval, self.tracking_busy_monitor)
    self.idle_timer = self.create_timer(self.timer_idle_interval, self.tracking_idle_monitor)

    #self.pub_environment_data = self.create_publisher(ObserverDayNight, 'bob/observer/day_night_classifier', publisher_qos_profile)

    # setup services, publishers and subscribers    
    self.sub_detector_state = self.create_subscription(DetectorState, 'bob/detection/detector_state', self.detector_state_callback, subscriber_qos_profile)
    self.sub_tracking_state = self.create_subscription(TrackingState, 'bob/tracker/tracking_state', self.tracking_state_callback, subscriber_qos_profile)
    self.sub_recording_state = self.create_subscription(RecordingState, 'bob/recording/recording_state', self.recording_state_callback, subscriber_qos_profile)
    self.bgs_reset_client = self.create_client(BGSResetRequest, 'bob/bgs/reset')

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def tracking_state_callback(self, msg_tracking_state:TrackingState):
    self.msg_tracking_state = msg_tracking_state

  def recording_state_callback(self, msg_recording_state:RecordingState):
    self.msg_recording_state = msg_recording_state
    #self.get_logger().info(f'{self.get_name()} RECORDING STATE: {self.msg_recording_state}')

  def detector_state_callback(self, msg_detector_state:DetectorState):
    self.msg_detector_state = msg_detector_state
    if self.msg_detector_state.maxblobsreached:
      self.get_logger().warn(f'Max blobs reached - please check detection mask or lower sensitivity level.')
      #request = BGSResetRequest.Request()
      #request.bgs_params = "test-service"
      #self.bgs_reset_client.call(request)

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
          self.get_logger().warn(f'Tracker needs tuning to a LOWER sensitivity level as its breaching the threshold of: {self.tracking_profile_high_switch_threshold} - Moving Average: {ma}')
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
          self.get_logger().warn(f'Tracker should be tuned to a HIGHER sensitivity level if possible - Moving Average: {ma}')

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