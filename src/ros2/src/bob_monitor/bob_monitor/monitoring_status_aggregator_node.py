import traceback as tb
import rclpy
import numpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from bob_shared.node_runner import NodeRunner
from bob_interfaces.msg import Tracking, RecordingState, DetectorState, MonitoringStatus, ObserverDayNight, ObserverCloudEstimation

class MonitoringStatusAggregatorNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('bob_monitoring_status_aggregator_node')

    self.declare_parameters(
      namespace='',
      parameters=
      [('observer_tracker_aggregation_interval', 1)])

    self.observer_tracker_aggregation_interval = self.get_parameter('observer_tracker_aggregation_interval').value

    self.msg_tracking_state = None
    self.msg_recording_state = None
    self.msg_detector_state = None
    self.msg_day_night_classification = None
    self.msg_cloud_estimation = None

    self.aggregation_timer = self.create_timer(self.observer_tracker_aggregation_interval, self.monitoring_status_publisher)

    # setup services, publishers and subscribers    
    self.sub_detector_state = self.create_subscription(DetectorState, 'bob/detection/detector_state', self.detector_state_callback, subscriber_qos_profile)
    self.sub_tracking_state = self.create_subscription(Tracking, 'bob/tracker/tracking', self.tracking_state_callback, subscriber_qos_profile)
    self.sub_recording_state = self.create_subscription(RecordingState, 'bob/recording/state', self.recording_state_callback, subscriber_qos_profile)

    self.sub_environment_day_night = self.create_subscription(ObserverDayNight, 'bob/observer/day_night_classifier', 
      self.day_night_callback, subscriber_qos_profile)
    self.sub_recording_state = self.create_subscription(ObserverCloudEstimation, 'bob/observer/cloud_estimation', 
      self.cloud_estimation_callback, subscriber_qos_profile)

    self.pub_aggregation_state = self.create_publisher(MonitoringStatus, 'bob/monitoring/status', publisher_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def tracking_state_callback(self, msg_tracking:Tracking):
    self.msg_tracking_state = msg_tracking.state

  def recording_state_callback(self, msg_recording_state:RecordingState):
    self.msg_recording_state = msg_recording_state
    #self.get_logger().info(f'{self.get_name()} RECORDING STATE: {self.msg_recording_state}')

  def detector_state_callback(self, msg_detector_state:DetectorState):
    self.msg_detector_state = msg_detector_state

  def day_night_callback(self, msg_day_night_classification:ObserverDayNight):
    self.msg_day_night_classification = msg_day_night_classification

  def cloud_estimation_callback(self, msg_cloud_estimation:ObserverCloudEstimation):
    self.msg_cloud_estimation = msg_cloud_estimation

  def monitoring_status_publisher(self):

    monitoring_status_msg = MonitoringStatus()

    monitoring_status_msg.trackable = 0
    monitoring_status_msg.alive = 0
    monitoring_status_msg.started = 0
    monitoring_status_msg.ended = 0
    monitoring_status_msg.sensitivity = ""
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
      monitoring_status_msg.sensitivity = self.msg_detector_state.sensitivity
      monitoring_status_msg.max_blobs_reached = self.msg_detector_state.max_blobs_reached
    
    if self.msg_recording_state is not None:
      monitoring_status_msg.recording = self.msg_recording_state.recording

    if self.msg_day_night_classification is not None:
      monitoring_status_msg.day_night_enum = self.msg_day_night_classification.day_night_enum
      monitoring_status_msg.avg_brightness = self.msg_day_night_classification.avg_brightness

    if self.msg_cloud_estimation is not None:
      monitoring_status_msg.percentage_cloud_cover = self.msg_cloud_estimation.percentage_cloud_cover
      monitoring_status_msg.unimodal_cloud_cover = self.msg_cloud_estimation.unimodal_cloud_cover

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

  node = MonitoringStatusAggregatorNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()