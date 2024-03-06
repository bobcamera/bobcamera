import rclpy.logging
from rclpy.time import Time
from datetime import datetime
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import Image
from bob_interfaces.msg import TrackingState, TrackDetectionArray, TrackTrajectoryArray

class RosbagRecorder():

  def __init__(self):
    self.logger = rclpy.logging.get_logger('rosbag_recorder')

    # setup the rosbag so that we can record into it
    db_uri = f'recordings/rosbags/{datetime.now().strftime("%Y_%m_%d-%H_%M_%S")}'
    self.logger.info(f'Creating database {db_uri}.')

    self.writer = rosbag2_py.SequentialWriter()
    
    storage_options = rosbag2_py._storage.StorageOptions(uri=db_uri, storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    self.writer.open(storage_options, converter_options)

    image_topic_info = rosbag2_py._storage.TopicMetadata(name='bob/frames/allsky/original', type='sensor_msgs/msg/Image', serialization_format='cdr')
    tracking_state_topic_info = rosbag2_py._storage.TopicMetadata(name='bob/tracker/tracking_state', type='bob_interfaces/msg/TrackingState', serialization_format='cdr')
    detection_topic_info = rosbag2_py._storage.TopicMetadata(name='bob/tracker/detections', type='bob_interfaces/msg/TrackDetectionArray', serialization_format='cdr')
    trajectory_topic_info = rosbag2_py._storage.TopicMetadata(name='bob/tracker/trajectory', type='bob_interfaces/msg/TrackTrajectoryArray', serialization_format='cdr')
    prediction_topic_info = rosbag2_py._storage.TopicMetadata(name='bob/tracker/prediction', type='bob_interfaces/msg/TrackTrajectoryArray', serialization_format='cdr')

    self.writer.create_topic(image_topic_info)
    self.writer.create_topic(detection_topic_info)
    self.writer.create_topic(tracking_state_topic_info)
    self.writer.create_topic(trajectory_topic_info)
    self.writer.create_topic(prediction_topic_info)

  def record(self, masked_frame:Image, msg_tracking_state:TrackingState, msg_detection_array:TrackDetectionArray, 
    msg_trajectory_array:TrackTrajectoryArray, msg_prediction_array:TrackTrajectoryArray):
     
    ns = Time.from_msg(masked_frame.header.stamp).nanoseconds
    self.writer.write('bob/frames/allsky/original', serialize_message(masked_frame), ns)
    self.writer.write('bob/tracker/tracking_state', serialize_message(msg_tracking_state), ns)
    self.writer.write('bob/tracker/detections', serialize_message(msg_detection_array), ns)
    self.writer.write('bob/tracker/trajectory', serialize_message(msg_trajectory_array), ns)
    self.writer.write('bob/tracker/prediction', serialize_message(msg_prediction_array), ns)

    status_message = f"(BoB) Tracker Status: trackable:{msg_tracking_state.trackable}, alive:{msg_tracking_state.alive}, started:{msg_tracking_state.started}, ended:{msg_tracking_state.ended}"
    self.logger.debug(status_message)
