import traceback as tb
import os
import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from bob_interfaces.srv import Mask, MaskUpdate
from bob_shared.node_runner import NodeRunner

class MaskWebApiNode(Node):

  def __init__(self):
    super().__init__('bob_mask_webapi')

    self.declare_parameters(namespace='', parameters=[
      ('masks_folder', 'assets/masks'),
      ('width', 1920),
      ('height', 1080)])
    
    self.masks_folder = self.get_parameter('masks_folder').value
    self.mask_width = self.get_parameter('width').value
    self.mask_height = self.get_parameter('height').value

    self.get_logger().debug(f'Masks path {self.masks_folder}, width {self.mask_width}, height {self.mask_height}.')

    self.br = CvBridge()

    # setup services, publishers and subscribers
    self.get_mask_service = self.create_service(Mask, 'bob/webapi/mask/image', self.get_mask_callback)
    self.put_mask_service = self.create_service(MaskUpdate, 'bob/webapi/mask/update', self.put_mask_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def get_mask_callback(self, request, response):

    try:      
      mask_file_path = os.path.join(self.masks_folder, request.file_name)

      if os.path.exists(mask_file_path) == False:
        self.get_logger().error(f'Mask path {mask_file_path} does not exist.')

      mask_image = cv2.imread(mask_file_path, cv2.IMREAD_GRAYSCALE)
      response.mask = self.br.cv2_to_imgmsg(mask_image)
    except Exception as e:
      self.get_logger().error(f"Exception getting mask. Error: {e}.")
      self.get_logger().error(tb.format_exc())

    return response

  def put_mask_callback(self, request, response):

    try:
      self.write_mask_file(request, response)
    except Exception as e:
      self.get_logger().error(f"Exception during writing mask file. Error: {e}.")

    return response

  def write_mask_file(self, request, response):

    # explicitly set the encoding to match that from the GUI
    mask_image = self.br.imgmsg_to_cv2(request.mask, "mono8")

    mask_file_path = os.path.join(self.masks_folder, request.file_name)

    width, height = mask_image.shape

    if (width != self.mask_width and height != self.mask_height):
      self.get_logger().info(f'Mask being resized from --> to w: {width} -> {self.mask_width}, h: {height} -> {self.mask_height}.')
      mask_image = cv2.resize(mask_image, (self.mask_width, self.mask_height), interpolation = cv2.INTER_LINEAR)

    if (request.invert):
      mask_image = cv2.bitwise_not(mask_image)

    if os.path.exists(mask_file_path) == False:
      self.get_logger().info(f'Mask path {mask_file_path} does not exist, adding mask.')
    else:
      self.get_logger().info(f'Mask path {mask_file_path} does exist, overwriting.')

    response.success = cv2.imwrite(mask_file_path, mask_image)

    return response
  
def main(args=None):

  rclpy.init(args=args)

  node = MaskWebApiNode()

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()