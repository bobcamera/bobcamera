import os
import cv2
import shutil
import traceback as tb
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from bob_shared.node_runner import NodeRunner
from bob_shared.enumerations import DayNightEnum
from bob_interfaces.msg import ObserverDayNight
from .star_mask import StarMaskMaker

class StarMaskNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('bob_star_mask')

    self.declare_parameters(
      namespace='',
      parameters=
      [('observer_timer_interval', 30),
       ('observer_cross_correlation_threshold', 0.5),
       ('detection_mask_file', 'assets/masks/detection-mask.jpg'),
       ('detection_mask_backup_file', 'assets/masks/detection-mask-backup.jpg')])

    self.br = CvBridge()
    self.timer_interval = self.get_parameter('observer_timer_interval').value
    self.msg_image = None
    self.star_mask = StarMaskMaker.MaskMaker(self.get_parameter('observer_cross_correlation_threshold').value)
    self.day_night: DayNightEnum = DayNightEnum.Unknown

    self.detection_mask_file = self.get_parameter('detection_mask_file').value
    self.detection_mask_backup_file = self.get_parameter('detection_mask_backup_file').value

    self.timer = self.create_timer(self.timer_interval, self.star_mask_maker)

    # setup services, publishers and subscribers    
    self.sub_camera = self.create_subscription(Image, 'bob/observer_frame/source', self.camera_callback, subscriber_qos_profile)
    self.sub_environment_day_night = self.create_subscription(ObserverDayNight, 'bob/observer/day_night_classifier', 
      self.day_night_callback, subscriber_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def camera_callback(self, msg_image:Image):
    self.msg_image = msg_image

  def day_night_callback(self, msg_day_night:ObserverDayNight):
    self.day_night = DayNightEnum(msg_day_night.day_night_enum)
    
  def star_mask_maker(self):
    try:
        if self.msg_image != None:
          # Check the day/night condition
          match self.day_night:
            case DayNightEnum.Day:
              # if we have a backup detection mark then restore the original detection mask for day operation
              if os.path.exists(self.detection_mask_backup_file):
                os.remove(self.detection_mask_file)
                shutil.move(self.detection_mask_backup_file, self.detection_mask_file)
                self.get_logger().info(f'Day: Star mask found, updating to detection mask only.')  
            case DayNightEnum.Night:                    
              if os.path.exists(self.detection_mask_file):
                # if we have a detection mark, but not a backup, create a backup of the original detection mask
                if os.path.exists(self.detection_mask_backup_file) == False:                  
                  shutil.copy(self.detection_mask_file, self.detection_mask_backup_file)                
                mask = cv2.imread(self.detection_mask_backup_file, cv2.IMREAD_GRAYSCALE)
                inverted_binary_mask = self.star_mask.process_image(self.br.imgmsg_to_cv2(self.msg_image))
                inverted_binary_mask_resized = cv2.resize(inverted_binary_mask, (mask.shape[1], mask.shape[0]))
                combined_mask = cv2.bitwise_and(mask, inverted_binary_mask_resized)
                cv2.imwrite(self.detection_mask_file, combined_mask)
                self.get_logger().info("Night: Detection mask updated with star positions")
              else:
                self.get_logger().info("Night: Detection mask not found. Unable to create star mask.")
            case _:
                self.get_logger().info(f'Unknown Day/Night classifier, ignore for now')
    except Exception as e:
        self.get_logger().error(f"Exception during star mask creation. Error: {e}.")
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

  node = StarMaskNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()