import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from bob_shared.node_runner import NodeRunner
from .object_simulator import ObjectSimulator

class ObjectSimulatorNode(Node):

    def __init__(self, publisher_qos_profile: QoSProfile):
        super().__init__('bob_object_simulator')

        # declare parameters and set defaults
        self.declare_parameters(
            namespace="",
            parameters=[                
                ("height", 1080),
                ("width", 1920)
                ])

        simulation_settings = {}

        # grab parameters provided
        simulation_settings['height'] = self.get_parameter('height').value
        simulation_settings['width'] = self.get_parameter('width').value

        self.timer = None
        self.br = CvBridge()
        self.timer_interval = 0.05  # Update frame every 50ms (20 FPS)
        self.object_simulator = ObjectSimulator.Generator(simulation_settings)

        self.timer = self.create_timer(self.timer_interval, self.publish_frame)

        self.pub_frame = self.create_publisher(Image, 'bob/simulation/output_frame', publisher_qos_profile)
        self.get_logger().info(f'{self.get_name()} node is up and running.')

    def publish_frame(self):
        frame = self.object_simulator.generate_frame()
        msg_frame = self.br.cv2_to_imgmsg(frame, "bgr8")
        self.pub_frame.publish(msg_frame)


def main(args=None):

    rclpy.init(args=args)

    publisher_qos_profile = QoSProfile(depth=10)
    publisher_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
    publisher_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
    publisher_qos_profile.history = QoSHistoryPolicy.KEEP_LAST

    node = ObjectSimulatorNode(publisher_qos_profile)

    runner = NodeRunner(node)
    runner.run()

if __name__ == '__main__':
    main()
