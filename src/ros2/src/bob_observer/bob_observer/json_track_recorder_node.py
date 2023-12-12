import traceback as tb
import rclpy
import json
import message_filters
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray
from bob_shared.enumerations import TrackingStateEnum
from bob_shared.node_runner import NodeRunner
from bob_interfaces.msg import Tracking, TrackDetection, TrackTrajectory

class JsonTrackRecorderNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('bob_json_track_recorder')

    self.declare_parameters(
      namespace='',
      parameters=
      [('json_recorder_directory', '.')])

    self.json_recorder_directory = self.get_parameter('json_recorder_directory').value

    # setup services, publishers and subscribers
    self.sub_masked_frame = message_filters.Subscriber(self, Image, 'bob/camera/all_sky/bayer', qos_profile=subscriber_qos_profile)
    self.sub_tracking_state = message_filters.Subscriber(self, Tracking, 'bob/tracker/tracking', qos_profile=subscriber_qos_profile)

    # setup the time synchronizer and register the subscriptions and callback
    self.time_synchronizer = message_filters.TimeSynchronizer([self.sub_masked_frame, self.sub_tracking_state], 10)
    self.time_synchronizer.registerCallback(self.synced_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def synced_callback(self, msg_frame: Image, msg_tracking: Tracking):
      if msg_frame is not None and msg_tracking is not None:
          try:
              time_in_secs = int(Time.from_msg(msg_frame.header.stamp).nanoseconds / 1e9)

              for i, (detection, trajectory) in enumerate(zip(msg_tracking.detections, msg_tracking.predictions)):
                  detection_id = detection.id
                  detection_state = TrackingStateEnum(int(detection.state))
                  bbox_msg: BoundingBox2D = detection.bbox
                  track_bbox_info = f'({bbox_msg.center.position.x},{bbox_msg.center.position.y},{bbox_msg.size_x},{bbox_msg.size_y})'

                  trajectory_info = 'No prediction'
                  if trajectory.trajectory:
                      most_recent_point = trajectory.trajectory[-1]
                      trajectory_info = f'id={trajectory.id}, center=({most_recent_point.center.x}, {most_recent_point.center.y})'

                  self.get_logger().info(
                      f'Time:[{time_in_secs}], ID:[{detection_id}], State:[{detection_state}], TraBB:[{track_bbox_info}], PredBB:[{trajectory_info}]'
                  )

          except Exception as e:
              self.get_logger().error(f"Exception during the annotated frame provider. Error: {e}.")
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

  node = JsonTrackRecorderNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()