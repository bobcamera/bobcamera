import traceback as tb
import os
import rclpy
import cv2
import cairosvg
from rclpy.node import Node
from cv_bridge import CvBridge
from bob_interfaces.srv import Mask, MaskGetSvg, MaskSvgUpdate, MaskSvgDelete
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
    self.get_mask_svg_service = self.create_service(MaskGetSvg, 'bob/webapi/mask/svg', self.get_mask_svg_callback)
    self.put_svg_mask_service = self.create_service(MaskSvgUpdate, 'bob/webapi/mask/update/svg', self.put_svg_mask_callback)
    self.delete_svg_mask_service = self.create_service(MaskSvgDelete, 'bob/webapi/mask/delete/svg', self.del_svg_mask_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')

  def get_mask_svg_callback(self, request, response):

    try:      
      mask_file_path = os.path.join(self.masks_folder, request.file_name)

      if os.path.exists(mask_file_path) == False:
        self.get_logger().error(f'Mask path {mask_file_path} does not exist.')

      with open(mask_file_path, 'r') as file:
        svg_content = file.read()
      response.mask = svg_content
      response.success = True

    except Exception as e:
      self.get_logger().error(f"Exception getting mask. Error: {e}.")
      self.get_logger().error(tb.format_exc())
      response.success = False

    return response

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

  def put_svg_mask_callback(self, request, response):

    try:
      self.write_svg_mask_file(request, response)
    except Exception as e:
      self.get_logger().error(f"Exception during SVG writing mask file. Error: {e}.")

    return response

  def del_svg_mask_callback(self, request, response):

    try:
      self.delete_mask_files(request, response)
    except Exception as e:
      self.get_logger().error(f"Exception during SVG writing mask file. Error: {e}.")

    return response

  def write_svg_mask_file(self, request, response):

    mask_file_path = os.path.join(self.masks_folder, request.file_name)

    if os.path.exists(mask_file_path) == False:
      self.get_logger().info(f'Mask path {mask_file_path} does not exist, adding mask.')
    else:
      self.get_logger().info(f'Mask path {mask_file_path} does exist, overwriting.')

    try:
      
      with open (mask_file_path, 'w') as file:  
        file.write(request.mask)

      input_svg_file = mask_file_path

      # Output PNG file path
      output_png_file = os.path.join(self.masks_folder, os.path.splitext(request.file_name)[0] + '.png')

      # Output JPG file path
      output_jpg_file = os.path.join(self.masks_folder, os.path.splitext(request.file_name)[0] + '.jpg')

      # Convert SVG to PNG
      cairosvg.svg2png(url=input_svg_file, write_to=output_png_file, output_width=self.mask_width, output_height=self.mask_height)

      # Convert 4 channel PNG to Grey image for mask application
      png_image = cv2.imread(output_png_file, cv2.IMREAD_UNCHANGED)
      grey_image = cv2.cvtColor(png_image, cv2.COLOR_RGBA2GRAY)
      cv2.imwrite(output_jpg_file, grey_image)
      
      response.success = True
    except Exception as e:
      self.get_logger().error(f"Exception during writing SVG mask file. Error: {e}.")
      response.success = False

    return response

  def delete_mask_files(self, request, response):

    svg_mask_file = os.path.join(self.masks_folder, request.file_name)
    png_mask_file = os.path.join(self.masks_folder, os.path.splitext(request.file_name)[0] + '.png')
    jpg_mask_file = os.path.join(self.masks_folder, os.path.splitext(request.file_name)[0] + '.jpg')

    try:

      if os.path.exists(svg_mask_file):
        os.remove(svg_mask_file)
        self.get_logger().info(f'SVG mask {svg_mask_file} exists, deleting.')

      if os.path.exists(png_mask_file):
        os.remove(png_mask_file)
        self.get_logger().info(f'PNG mask {png_mask_file} exists, deleting.')

      if os.path.exists(jpg_mask_file):
        os.remove(jpg_mask_file)
        self.get_logger().info(f'JPG mask {jpg_mask_file} exists, deleting.')                

      response.success = True
    except Exception as e:
      self.get_logger().error(f"Exception during writing SVG mask file. Error: {e}.")
      response.success = False

    return response

def main(args=None):

  rclpy.init(args=args)

  node = MaskWebApiNode()

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()