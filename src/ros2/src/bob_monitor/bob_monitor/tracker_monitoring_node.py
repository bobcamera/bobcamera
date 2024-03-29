import traceback as tb
import rclpy
import numpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from bob_shared.node_runner import NodeRunner
from bob_interfaces.msg import Tracking, RecordingState, DetectorState, MonitoringStatus, ObserverDayNight, ObserverCloudEstimation
from bob_interfaces.srv import BGSResetRequest
from bob_shared.enumerations import TrackingHintEnum

class TrackerMonitorNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('bob_tracker_monitor')

    self.declare_parameters(
      namespace='',
      parameters=
      [('observer_tracker_monitor_busy_interval', 5),
       ('observer_tracker_monitor_idle_interval', 60),
       ('observer_tracker_aggregation_interval', 1),
       ('observer_tracking_profile_busy_switch_threshold', 5),
       ('observer_tracker_sample_set', 5)])

    self.timer_busy_interval = self.get_parameter('observer_tracker_monitor_busy_interval').value
    self.timer_idle_interval = self.get_parameter('observer_tracker_monitor_idle_interval').value
    self.observer_tracker_aggregation_interval = self.get_parameter('observer_tracker_aggregation_interval').value
    self.tracking_profile_high_switch_threshold = self.get_parameter('observer_tracking_profile_busy_switch_threshold').value
    self.sample_set = self.get_parameter('observer_tracker_sample_set').value

    self.msg_tracking_state = None
    self.msg_recording_state = None
    self.msg_detector_state = None
    self.msg_day_night_classification = None
    self.msg_cloud_estimation = None
    self.tracking_hint:TrackingHintEnum = TrackingHintEnum.NoHint
    
    self.avg_tracks = [0]*self.sample_set
    self.track_counter = 0
    self.started = 0
    self.idle_enabled = False

    self.busy_timer = self.create_timer(self.timer_busy_interval, self.tracking_busy_monitor)
    self.idle_timer = self.create_timer(self.timer_idle_interval, self.tracking_idle_monitor)
    self.aggregation_timer = self.create_timer(self.observer_tracker_aggregation_interval, self.monitoring_status_publisher)

    # setup services, publishers and subscribers    
    self.sub_detector_state = self.create_subscription(DetectorState, 'bob/detection/detector_state', self.detector_state_callback, subscriber_qos_profile)
    self.sub_tracking_state = self.create_subscription(Tracking, 'bob/tracker/tracking', self.tracking_state_callback, subscriber_qos_profile)
    self.sub_recording_state = self.create_subscription(RecordingState, 'bob/recording/recording_state', self.recording_state_callback, subscriber_qos_profile)

    self.sub_environment_day_night = self.create_subscription(ObserverDayNight, 'bob/observer/day_night_classifier', 
      self.day_night_callback, subscriber_qos_profile)
    self.sub_recording_state = self.create_subscription(ObserverCloudEstimation, 'bob/observer/cloud_estimation', 
      self.cloud_estimation_callback, subscriber_qos_profile)

    self.bgs_reset_client = self.create_client(BGSResetRequest, 'bob/bgs/reset')

    self.pub_aggregation_state = self.create_publisher(MonitoringStatus, 'bob/monitoring/status', publisher_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def tracking_state_callback(self, msg_tracking:Tracking):
    self.msg_tracking_state = msg_tracking.state

  def recording_state_callback(self, msg_recording_state:RecordingState):
    self.msg_recording_state = msg_recording_state
    #self.get_logger().info(f'{self.get_name()} RECORDING STATE: {self.msg_recording_state}')

  def detector_state_callback(self, msg_detector_state:DetectorState):
    self.msg_detector_state = msg_detector_state
    if self.msg_detector_state.max_blobs_reached:
      self.get_logger().warn(f'Max blobs reached - please check detection mask or lower sensitivity level.')
      #request = BGSResetRequest.Request()
      #request.bgs_params = "test-service"
      #self.bgs_reset_client.call(request)

  def day_night_callback(self, msg_day_night_classification:ObserverDayNight):
    self.msg_day_night_classification = msg_day_night_classification

  def cloud_estimation_callback(self, msg_cloud_estimation:ObserverCloudEstimation):
    self.msg_cloud_estimation = msg_cloud_estimation

  def tracking_busy_monitor(self):

    if self.msg_tracking_state != None:

      try:
        test = 0
        # # what is the delta tracks for the last second
        # delta = int(self.msg_tracking_state.started) - self.started
        # self.started = int(self.msg_tracking_state.started)

        # self.avg_tracks[self.track_counter] = delta
        # self.track_counter += 1
        # if self.track_counter >= self.sample_set:
        #   self.track_counter = 0

        # ma = numpy.sum(self.avg_tracks, dtype=numpy.int32) / self.sample_set

        # #self.get_logger().info(f'{self.get_name()} BUSY ma: {ma}')

        # if ma > self.tracking_profile_high_switch_threshold:
        #   self.tracking_hint:TrackingHintEnum = TrackingHintEnum.LowerSensitivity
        #   self.get_logger().warn(f'Tracker needs tuning to a LOWER sensitivity level as its breaching the threshold of: {self.tracking_profile_high_switch_threshold} - Moving Average: {ma}')
        #   self.avg_tracks = [0]*self.sample_set # Reset the avg_tracks variable otherwise we might get several BUSY notifications

      except Exception as e:
        self.get_logger().error(f"Exception during alive track monitor. Error: {e}.")
        self.get_logger().error(tb.format_exc())
  
  def tracking_idle_monitor(self):

    if self.msg_tracking_state != None and self.msg_cloud_estimation is not None:

      try:
        test = 0
        # what is the delta tracks for the last second
        # delta = int(self.msg_tracking_state.started) - self.started
        # self.started = int(self.msg_tracking_state.started)

        # self.avg_tracks[self.track_counter] = delta
        # self.track_counter += 1
        # if self.track_counter >= self.sample_set:
        #   self.track_counter = 0
        #   self.idle_enabled = True

        # ma = numpy.sum(self.avg_tracks, dtype=numpy.int32) / self.sample_set

        # #self.get_logger().info(f'{self.get_name()} IDLE ma: {ma}')

        # if self.idle_enabled and ma == 0:
        #   self.tracking_hint:TrackingHintEnum = TrackingHintEnum.IncreaseSensitivity
        #   self.get_logger().warn(f'Tracker should be tuned to a HIGHER sensitivity level if possible - Moving Average: {ma}')
        
        if self.msg_cloud_estimation.unimodal_cloud_cover == True:
          self.tracking_hint:TrackingHintEnum = TrackingHintEnum.IncreaseSensitivity
        elif self.msg_cloud_estimation.unimodal_cloud_cover == False:
          self.tracking_hint:TrackingHintEnum = TrackingHintEnum.LowerSensitivity
        else:
          self.tracking_hint:TrackingHintEnum = TrackingHintEnum.NoHint

      except Exception as e:
        self.get_logger().error(f"Exception during alive track monitor. Error: {e}.")
        self.get_logger().error(tb.format_exc())

  def monitoring_status_publisher(self):

    monitoring_status_msg = MonitoringStatus()

    monitoring_status_msg.trackable = 0
    monitoring_status_msg.alive = 0
    monitoring_status_msg.started = 0
    monitoring_status_msg.ended = 0
    monitoring_status_msg.max_blobs_reached = False
    monitoring_status_msg.recording = False    
    monitoring_status_msg.day_night_enum = 0
    monitoring_status_msg.avg_brightness = 0
    monitoring_status_msg.percentage_cloud_cover = 0
    monitoring_status_msg.unimodal_cloud_cover = None

    if self.msg_tracking_state is not None:
      monitoring_status_msg.trackable = self.msg_tracking_state.trackable
      monitoring_status_msg.alive = self.msg_tracking_state.alive
      monitoring_status_msg.started = self.msg_tracking_state.started
      monitoring_status_msg.ended = self.msg_tracking_state.ended
    
    if self.msg_detector_state is not None:
      monitoring_status_msg.max_blobs_reached = self.msg_detector_state.max_blobs_reached
    
    if self.msg_recording_state is not None:
      monitoring_status_msg.recording = self.msg_recording_state.recording

    if self.msg_day_night_classification is not None:
      monitoring_status_msg.day_night_enum = self.msg_day_night_classification.day_night_enum
      monitoring_status_msg.avg_brightness = self.msg_day_night_classification.avg_brightness

    if self.msg_cloud_estimation is not None:
      monitoring_status_msg.percentage_cloud_cover = self.msg_cloud_estimation.percentage_cloud_cover
      monitoring_status_msg.unimodal_cloud_cover = self.msg_cloud_estimation.unimodal_cloud_cover

    # monitoring_status_msg.tracking_hint = 'None'
    # if self.tracking_hint == TrackingHintEnum.IncreaseSensitivity:
    #   monitoring_status_msg.tracking_hint = 'Increase Sensitivity'
    # elif self.tracking_hint == TrackingHintEnum.LowerSensitivity:
    #   monitoring_status_msg.tracking_hint = 'Lower Sensitivity or Improve Detection Mask'
    # else:
    #   if monitoring_status_msg.max_blobs_reached:
    #     monitoring_status_msg.tracking_hint = 'Lower Sensitivity or Improve Detection Mask'

    monitoring_status_msg.tracking_hint = 'None'
    if self.tracking_hint == TrackingHintEnum.IncreaseSensitivity:
      monitoring_status_msg.tracking_hint = 'Stable conditions - set to medium or high sensitivity'
    elif self.tracking_hint == TrackingHintEnum.LowerSensitivity:
      monitoring_status_msg.tracking_hint = 'Unstable conditions - set to low sensitivity'
    else:
      if monitoring_status_msg.max_blobs_reached:
        monitoring_status_msg.tracking_hint = 'Max blobs reached - lower Sensitivity or check detection mask'
    
    self.pub_aggregation_state.publish(monitoring_status_msg)

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