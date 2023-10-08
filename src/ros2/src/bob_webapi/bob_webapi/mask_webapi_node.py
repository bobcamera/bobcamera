import traceback as tb
import os
import rclpy
import cv2
from rclpy.node import Node
from typing import List
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from bob_interfaces.srv import Mask, MaskUpdate
from bob_shared.node_runner import NodeRunner

class MaskWebApiNode(Node):

  def __init__(self):
    super().__init__('bob_mask_webapi')

    self.br = CvBridge()

    # setup services, publishers and subscribers
    self.get_mask_service = self.create_service(Mask, 'bob/webapi/mask/image', self.get_mask_callback)
    self.put_mask_service = self.create_service(MaskUpdate, 'bob/webapi/mask/update', self.put_mask_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def get_mask_callback(self, request, response):

    try:
      masks_folder = self.videos_folder = os.path.join(get_package_share_directory('bob_webapi'), 'masks')
      mask_file_path = os.path.join(masks_folder, request.file_name)

      if os.path.exists(mask_file_path) == False:
        self.get_logger().error(f'Mask path {request.file_name} does not exist.')

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

    mask_image = self.br.imgmsg_to_cv2(request.mask)
    masks_folder = self.videos_folder = os.path.join(get_package_share_directory('bob_webapi'), 'masks')
    mask_file_path = os.path.join(masks_folder, request.file_name)

    if os.path.exists(mask_file_path) == False:
      self.get_logger().info(f'Mask path {request.file_name} does not exist, adding mask.')
    else:
      self.get_logger().info(f'Mask path {request.file_name} does exist, overwriting.')

    response.success = cv2.imwrite(mask_file_path, mask_image)

    return response
  
def main(args=None):

  rclpy.init(args=args)

  node = MaskWebApiNode()

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()