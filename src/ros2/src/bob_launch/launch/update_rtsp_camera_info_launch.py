import os
import yaml
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration 
from ament_index_python.packages import get_package_share_directory

def launch_setup(context):

    # get the values provided as part of the launch arguments
    width = int(LaunchConfiguration('rtsp_width_arg').perform(context))
    height = int(LaunchConfiguration('rtsp_height_arg').perform(context))

    # Get the config directory
    config_dir = os.path.join(get_package_share_directory('bob_launch'), 'config')
    config_file = os.path.join(config_dir, "camera_info.yaml")
    with open(config_file, 'r') as read:
        yaml_output = yaml.safe_load(read)
        yaml_output['image_width'] = width
        yaml_output['image_height'] = height
    
    # Update the camera_info file with the provided launch arguments
    with open(config_file, 'w') as write:
        yaml.dump(yaml_output, write, sort_keys = False)

def generate_launch_description():    
    opaqueFunction = OpaqueFunction(function = launch_setup)
    #opaqueFunction.condition=IfCondition(PythonExpression([LaunchConfiguration('source_arg'), " == 'rtsp' or 'rtsp_overlay'" ]))

    return LaunchDescription([
        opaqueFunction
    ])




