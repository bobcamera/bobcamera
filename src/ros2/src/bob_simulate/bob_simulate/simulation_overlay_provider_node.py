import traceback as tb
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import List
from sensor_msgs.msg import Image
from bob_shared.node_runner import NodeRunner
from .synthetic_data import DroneSyntheticData, PlaneSyntheticData
from .simulation_test import SimulationTest
from .simulation_test_case import SimulationTestCase
from .simulation_test_case_runner import SimulationTestCaseRunner

class SimulationOverlayProviderNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('bob_overlayed_video_provider')

    # setup services, publishers and subscribers
    self.sub_camera = self.create_subscription(Image, 'bob/simulation/input_frame', self.camera_callback, subscriber_qos_profile)
    self.pub_synthetic_frame = self.create_publisher(Image, 'bob/simulation/output_frame', publisher_qos_profile)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def camera_callback(self, msg_image:Image):

    if not msg_image is None:

      if self.test_case_runner.active:

        try:
          input_frame = self.br.imgmsg_to_cv2(msg_image)
          frame_synthetic = self.test_case_runner.run(input_frame)

          frame_synthetic_msg = self.br.cv2_to_imgmsg(frame_synthetic, encoding=msg_image.encoding)
          frame_synthetic_msg.header = msg_image.header

          self.pub_synthetic_frame.publish(frame_synthetic_msg)        
        except Exception as e:
          self.get_logger().error(f"Exception during frame overlay simulation. Error: {e}.")
          self.get_logger().error(tb.format_exc())
      
      else:
        self.get_logger().info(f'{self.get_name()} Overlay simulation complete.')

  def config_list(self) -> List[str]:
    return ['frame_provider_resize_dimension_h', 'frame_provider_resize_dimension_w']

  def validate_config(self) -> bool:
    valid = True
    return valid

  def on_config_loaded(self, init: bool):
    if init:
      self.br = CvBridge()

      self.test_case_runner = SimulationTestCaseRunner(
        [
          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=3, loop=False)], (960, 960), simulation_name='Drone'),
          #SimulationTestCase(self, [SimulationTest(PlaneSyntheticData(), target_object_diameter=3, loop=False)], (960, 960), simulation_name='Plane'),
          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=3, loop=False), SimulationTest(PlaneSyntheticData(), target_object_diameter=3, loop=False)], (960, 960), simulation_name='Drone & Plane'),

          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=5, loop=False)], (1440, 1440), simulation_name='Drone'),
          #SimulationTestCase(self, [SimulationTest(PlaneSyntheticData(), target_object_diameter=5, loop=True)], (1440, 1440), simulation_name='Plane'),
          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=5, loop=False), SimulationTest(PlaneSyntheticData(), target_object_diameter=5, loop=False)], (1440, 1440), simulation_name='Drone & Plane'),
          
          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=8, loop=False)], (2160, 2160), simulation_name='Drone'),
          #SimulationTestCase(self, [SimulationTest(PlaneSyntheticData(), target_object_diameter=8, loop=False)], (2160, 2160), simulation_name='Plane'),
          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=8, loop=False), SimulationTest(PlaneSyntheticData(), target_object_diameter=8, loop=False)], (2160, 2160), simulation_name='Drone & Plane'),

          SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=11, loop=False)], (2880, 2880), simulation_name='Drone'),
          SimulationTestCase(self, [SimulationTest(PlaneSyntheticData(), target_object_diameter=11, loop=False)], (2880, 2880), simulation_name='Plane'),
          SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=11, loop=False), SimulationTest(PlaneSyntheticData(), target_object_diameter=11, loop=False)], (2880, 2880), simulation_name='Drone & Plane'),

          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=15, loop=False)], (3600, 3600), simulation_name='Drone'),
          #SimulationTestCase(self, [SimulationTest(PlaneSyntheticData(), target_object_diameter=15, loop=False)], (3600, 3600), simulation_name='Plane'),       
          #SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=15, loop=False), SimulationTest(PlaneSyntheticData(), target_object_diameter=15, loop=False)], (3600, 3600), simulation_name='Drone & Plane'),
        ]
      )

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

  node = SimulationOverlayProviderNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()
