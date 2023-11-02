import traceback as tb
import rclpy
from rclpy.node import Node
from bob_interfaces.srv import ApplicationVersion
from bob_shared.node_runner import NodeRunner

class VersionWebApiNode(Node):

  def __init__(self):
    super().__init__('bob_version_webapi')

    self.declare_parameters(namespace='', parameters=[('application_version', '0.0.0')])
    
    self.application_version = self.get_parameter('application_version').value

    self.get_logger().info(f'BOB Application Version Number: {self.application_version}.')

    # setup services, publishers and subscribers
    self.get_mask_service = self.create_service(ApplicationVersion, 'bob/webapi/application/version', self.get_version_number_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def get_version_number_callback(self, request, response):

    try:      
      response.version = self.application_version
    except Exception as e:
      self.get_logger().error(f"Exception getting application version number. Error: {e}.")
      self.get_logger().error(tb.format_exc())

    return response
  
def main(args=None):

  rclpy.init(args=args)

  node = VersionWebApiNode()

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()