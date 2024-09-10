import traceback as tb
import rclpy
import os
import yaml
import numpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from bob_interfaces.srv import ConfigEntryUpdate
from bob_shared.node_runner import NodeRunner

class ConfigManagerNode(Node):

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
    super().__init__('bob_tracker_monitor')

    self.declare_parameters(
      namespace='',
      parameters=
      [('config_file_path', 'assets/config/app_config.yaml')])

    self.app_config_file = self.get_parameter('config_file_path').value

    self.fps_srv = self.create_service(ConfigEntryUpdate, 'bob/config/update/fps', self.fps_update_callback)

    self.get_logger().info(f'{self.get_name()} node is up and running.')
   
  def fps_update_callback(self, request, response):

      try:

        self.get_logger().info(f"Successfully updated {request.key}-{request.type}-{request.value}.")

        fps = float(request.value)
          
        with open(self.app_config_file, 'r') as read:
          yaml_output = yaml.safe_load(read)

          # simulated_frame_provider_node
          yaml_output['simulated_frame_provider_node']['ros__parameters']['video_fps'] = fps

          # simulation_overlay_provider_node
          yaml_output['simulation_overlay_provider_node']['ros__parameters']['video_fps'] = fps

          # info_webapi_node
          yaml_output['info_webapi_node']['ros__parameters']['video_fps'] = fps

          # allsky_recorder_node
          yaml_output['allsky_recorder_node']['ros__parameters']['video_fps'] = fps

        # Update the camera_info file with the provided launch arguments
        with open(self.app_config_file, 'w') as write:
          yaml.Dumper.ignore_aliases = lambda *args: True
          yaml.dump(yaml_output, write, sort_keys = False, width=1080)

        response.success = True            
      except Exception as e:
        response.message = f"Error updating FPS: {e}"
        response.success = False
        self.get_logger().error(response.message)          

      return response

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

  node = ConfigManagerNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()


if __name__ == '__main__':
  main()