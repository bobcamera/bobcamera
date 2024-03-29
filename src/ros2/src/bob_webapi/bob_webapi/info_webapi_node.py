import traceback as tb
import rclpy
from rclpy.node import Node
from bob_interfaces.srv import ApplicationInfo
from bob_shared.node_runner import NodeRunner

class InfoWebApiNode(Node):

  def __init__(self):
    super().__init__('bob_info_webapi')

    self.declare_parameters(namespace='', parameters=[('application_version', '0.0.0')])
    self.declare_parameters(namespace='', parameters=[('frame_width', 1920)])
    self.declare_parameters(namespace='', parameters=[('frame_height', 1080)])
    self.declare_parameters(namespace='', parameters=[('video_fps', 15.0)])
    
    self.application_version = self.get_parameter('application_version').value
    self.frame_width = self.get_parameter('frame_width').value
    self.frame_height = self.get_parameter('frame_height').value
    self.video_fps = self.get_parameter('video_fps').value

    self.get_logger().info(
      f'Application Info - application_version: {self.application_version}, frame_width: {self.frame_width}, frame_height: {self.frame_height}, video_fps: {self.video_fps}.')

    # setup services, publishers and subscribers
    self.get_mask_service = self.create_service(ApplicationInfo, 'bob/webapi/application/info', self.get_info_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def get_info_callback(self, request, response):

    try:      
      response.version = self.application_version
      response.frame_width = self.frame_width
      response.frame_height = self.frame_height
      response.video_fps = self.video_fps
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