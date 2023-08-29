from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bob_camera',
            executable='qhy_node',
            name='all_sky_publisher_node',
            output='screen',
            parameters=[{'camera_id': ''}
                        , {'image_publish_topic': 'bob/camera/all_sky/bayer'}
                        , {'image_info_publish_topic': 'bob/camera/all_sky/image_info'}
                        , {'camera_info_publish_topic': 'bob/camera/all_sky/camera_info'}
                        , {'enable_profiling': False}
                        , {'exposure': 20000}
                        , {'gain': 30}
                        , {'auto_exposure': True}]

        ),
        Node(
            package='bob_image_processing',
            executable='background_subtractor_node',
            name='background_subtractor_node',
            output='screen',
            parameters=[{'enable_profiling': False}]
        ),
        # Node(
        #     package='bob_tracking',
        #     executable='track_provider_node',
        #     name='track_provider_node',
        #     output='screen',
        #     parameters=[{'enable_profiling': True}]
        # ),
        # Node(
        #     package='bob_image_processing',
        #     executable='annotated_frame_provider_node',
        #     name='annotated_frame_provider_node',
        #     output='screen'
        # ),
        # Node(
        #     package='bob_visualizers',
        #     executable='frame_viewer_node',
        #     name='frame_viewer_node',
        #     output='screen',
        #     parameters=[{"topics": ["bob/frames/annotated", "bob/camera/all_sky/bayer", "bob/frames/all_sky/foreground_mask"]}]
        # ),
        Node(
            package='bob_visualizers',
            executable='frame_bbox_viewer_node',
            name='frame_bbox_viewer_node',
            output='screen',
            parameters=[{"topics": ["bob/camera/all_sky/bayer", "bob/frames/all_sky/foreground_mask"]}]
        ),
    ])
