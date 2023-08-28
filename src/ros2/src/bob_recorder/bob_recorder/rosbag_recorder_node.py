# Original work Copyright (c) 2022 Sky360
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

import traceback as tb
import rclpy
import message_filters
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy, QoSHistoryPolicy
from rclpy.executors import ExternalShutdownException
from typing import List
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from bob_interfaces.msg import TrackingState, TrackDetectionArray, TrackTrajectoryArray
from .rosbag_recorder import RosbagRecorder

class RosbagRecorderNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile):
    super().__init__('rosbag_recorder')

    self.msg_frame: Image = None
    self.msg_tracking_state: TrackingState = None
    self.br = CvBridge()

    self.recorder = RosbagRecorder()

    self.sub_masked_frame = message_filters.Subscriber(self, Image, 'bob/camera/all_sky/bayer', qos_profile=subscriber_qos_profile)
    self.sub_tracking_state = message_filters.Subscriber(self, TrackingState, 'bob/tracker/tracking_state', qos_profile=subscriber_qos_profile)    
    self.sub_tracker_detections = message_filters.Subscriber(self, TrackDetectionArray, 'bob/tracker/detections', qos_profile=subscriber_qos_profile)
    self.sub_tracker_trajectory = message_filters.Subscriber(self, TrackTrajectoryArray, 'bob/tracker/trajectory', qos_profile=subscriber_qos_profile)
    self.sub_tracker_prediction = message_filters.Subscriber(self, TrackTrajectoryArray, 'bob/tracker/prediction', qos_profile=subscriber_qos_profile)

    # setup the time synchronizer and register the subscriptions and callback
    self.time_synchronizer = message_filters.TimeSynchronizer([self.sub_masked_frame, self.sub_tracking_state, 
      self.sub_tracker_detections, self.sub_tracker_trajectory, self.sub_tracker_prediction], 10)
    self.time_synchronizer.registerCallback(self.synced_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def synced_callback(self, masked_frame:Image, msg_tracking_state:TrackingState, msg_detection_array:TrackDetectionArray, 
    msg_trajectory_array:TrackTrajectoryArray, msg_prediction_array:TrackTrajectoryArray):

    if masked_frame is not None and msg_tracking_state is not None and msg_detection_array is not None and msg_trajectory_array is not None:      
      if msg_tracking_state.trackable > 0:

        try:
          self.recorder.record(masked_frame, msg_tracking_state, msg_detection_array, msg_trajectory_array, msg_prediction_array)
        except Exception as e:
          self.get_logger().error(f"Exception during activity recorder. Error: {e}.")
          self.get_logger().error(tb.format_exc())


def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = QoSProfile(depth=1)
  subscriber_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
  subscriber_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
  subscriber_qos_profile.history = QoSHistoryPolicy.KEEP_LAST

  node = RosbagRecorderNode(subscriber_qos_profile)

  try:
      rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
      pass
  finally:
      node.destroy_node()
      rclpy.try_shutdown()

if __name__ == '__main__':
  main()