import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

class NodeRunner():

    def __init__(self, node: Node):
        self.node = node

    def run(self):

        try:
            #self.node.get_logger().info(f'Running via from node runner')
            #rclpy.logging.get_logger('node_runner').info(f'Running node {self.node.get_name()} via the node runner')
            rclpy.spin(self.node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            #self.node.get_logger().info(f'Shutting down node from node runner')
            #rclpy.logging.get_logger('node_runner').info(f'Shutting down {self.node.get_name()} via the node runner')
            self.node.destroy_node()
            rclpy.try_shutdown()
