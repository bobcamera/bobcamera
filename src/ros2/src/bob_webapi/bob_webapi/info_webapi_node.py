import traceback as tb
import rclpy
from rclpy.node import Node
from bob_interfaces.srv import ApplicationInfo
from bob_shared.node_runner import NodeRunner

class InfoWebApiNode(Node):

  def __init__(self):
    super().__init__('bob_info_webapi')

    self.declare_parameters(namespace='', parameters=[('version_file', 'version.txt')])

    with open(self.get_parameter('version_file').value, 'r') as file:
      self.application_version = file.read()
    
    self.get_logger().info(
      f'Application Info - application_version: {self.application_version}')

    # setup services, publishers and subscribers
    self.get_mask_service = self.create_service(ApplicationInfo, 'bob/webapi/application/info', self.get_info_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def get_info_callback(self, request, response):

    try:      
      response.version = self.application_version
    except Exception as e:
      self.get_logger().error(f"Exception getting application information. Error: {e}.")
      self.get_logger().error(tb.format_exc())

    return response
  
def main(args=None):

  rclpy.init(args=args)

  node = InfoWebApiNode()

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()