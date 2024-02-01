import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration 
from launch.substitutions import PythonExpression, LaunchConfiguration 
from launch.conditions import IfCondition

def generate_launch_description():

    # Get the application config
    #config = os.path.join(get_package_share_directory('bob_launch'), 'config', 'app_config.yaml')
    config = 'assets/config/app_config.yaml'

    AbsoluteMoveNode = Node(
        package='bob_ptz',
        #namespace='bob',
        executable='onvif_absolute_move_ros',
        name='AbsoluteMoveNode',
        parameters = [config],
        #condition=IfCondition(PythonExpression([
        #            LaunchConfiguration('source_arg'), 
        #            " == 'rtsp'",
        #            " or ", 
        #            LaunchConfiguration('source_arg'), 
        #            " == 'rtsp_overlay'"])),
    )

    RasterImageAcquisitionNode = Node(
        package='bob_ptz',
        executable='raster_image_acquisition_service',  # Replace with your actual script name
        name='RasterImageAcquisitionService',
        parameters=[config],
        #condition=IfCondition(PythonExpression([
        #    LaunchConfiguration('source_arg'),
        #    " == 'rtsp'",
        #    " or ",
        #    LaunchConfiguration('source_arg'),
        #    " == 'rtsp_overlay'"
        #])),
    )

    ImageAcquisitionClientNode = Node(
        package='bob_ptz',
        executable='raster_ptz_client',  # Replace with your actual script name
        name='RasterPTZClient',
        #condition=IfCondition(PythonExpression([
        #    LaunchConfiguration('source_arg'),
        #    " == 'rtsp'",
        #    " or ",
        #    LaunchConfiguration('source_arg'),
        #    " == 'rtsp_overlay'"
        #])),
    )

    return LaunchDescription([
        AbsoluteMoveNode,
        RasterImageAcquisitionNode,
        ImageAcquisitionClientNode,
    ])

