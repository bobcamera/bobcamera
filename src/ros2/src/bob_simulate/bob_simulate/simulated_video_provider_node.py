import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from bob_shared.node_runner import NodeRunner
from .synthetic_data import DroneSyntheticData, PlaneSyntheticData
from .simulation_test import SimulationTest
from .simulation_test_case import SimulationTestCase
from .simulation_test_case_runner import SimulationTestCaseRunner

class SimulatedVideoProviderNode(Node):

  def __init__(self, publisher_qos_profile: QoSProfile):
    super().__init__('bob_simulated_video_provider')

    self.timer_period = 0.03
    self.br = CvBridge()
    self.still_frame_image = None

    self.declare_parameters(
        namespace="",
        parameters=[                
            ("height", 1080),
            ("width", 1920),
            ("target_object_diameter", 5)])

    # grab parameters provided
    self.height = self.get_parameter('height').value
    self.width = self.get_parameter('width').value
    self.target_object_diameter = self.get_parameter('target_object_diameter').value

    self.test_case_runner = SimulationTestCaseRunner([
      SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=self.target_object_diameter, loop=False)], (self.width, self.height), simulation_name='Drone'),
      SimulationTestCase(self, [SimulationTest(PlaneSyntheticData(), target_object_diameter=self.target_object_diameter, loop=False)], (self.width, self.height), simulation_name='Plane'),
      SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=self.target_object_diameter, loop=False), SimulationTest(PlaneSyntheticData(), target_object_diameter=5, loop=False)], (self.width, self.height), simulation_name='Drone & Plane'),
    ])

    # setup services, publishers and subscribers
    self.pub_synthetic_frame = self.create_publisher(Image, 'bob/simulation/output_frame', publisher_qos_profile)

    self.timer = self.create_timer(self.timer_period, self.timer_callback)  

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def timer_callback(self):

    if self.test_case_runner.active:

      frame_synthetic = self.test_case_runner.run()

      frame_synthetic_msg = self.br.cv2_to_imgmsg(frame_synthetic, encoding="bgr8")
      frame_synthetic_msg.header.stamp = self.get_time_msg()
      frame_synthetic_msg.header.frame_id = 'synthetic'

      self.pub_synthetic_frame.publish(frame_synthetic_msg)

    else:
      self.get_logger().info(f'{self.get_name()} Simulation complete, restarting.')

      #self.test_case_runner = SimulationTestCaseRunner([
      #  SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=self.target_object_diameter, loop=False)], (self.width, self.height), simulation_name='Drone'),
      #  SimulationTestCase(self, [SimulationTest(PlaneSyntheticData(), target_object_diameter=self.target_object_diameter, loop=False)], (self.width, self.height), simulation_name='Plane'),
      #  SimulationTestCase(self, [SimulationTest(DroneSyntheticData(), target_object_diameter=self.target_object_diameter, loop=False), SimulationTest(PlaneSyntheticData(), target_object_diameter=5, loop=False)], (self.width, self.height), simulation_name='Drone & Plane'),
      #])      

  def get_time_msg(self):
    time_msg = Time()
    msg_time = self.get_clock().now().seconds_nanoseconds()

    time_msg.sec = int(msg_time[0])
    time_msg.nanosec = int(msg_time[1])
    return time_msg

def main(args=None):

  rclpy.init(args=args)

  publisher_qos_profile = QoSProfile(depth=10)
  publisher_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
  publisher_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
  publisher_qos_profile.history = QoSHistoryPolicy.KEEP_LAST

  node = SimulatedVideoProviderNode(publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()