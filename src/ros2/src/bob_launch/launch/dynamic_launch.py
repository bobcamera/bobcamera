import os
import launch
import yaml
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PythonExpression, LaunchConfiguration, EnvironmentVariable 
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def parse_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading YAML file: {e}")
        return None

def generate_composable_nodes(config, node_container, namespace, config_file):
    node_descriptions = node_container['nodes']
    nodes = []
    for node_description in node_descriptions:
        remappings = [
            (k, v) for mapping in node_description.get('remappings', [])
            for k, v in mapping.items()
        ]
        parameters = config[node_description['name']]['ros__parameters']

        print(f"Node: Package: {node_description['package']}, Plugin: {node_description['plugin']}, Name: {node_description['name']}")
        node = ComposableNode(
            package=node_description['package'],
            plugin=node_description['plugin'],
            name=node_description['name'],
            namespace=namespace,
            parameters = [parameters],
            extra_arguments=[{'use_intra_process_comms': True}],
            remappings=remappings
        )
        nodes.append(node)
    return nodes


def generate_launch_description():
    config_file_path = 'assets/config/video_simulator_config.yaml'
    namespace = EnvironmentVariable('BOB_NAMESPACE', default_value="")
    loglevel = EnvironmentVariable('BOB_LOGLEVEL', default_value="INFO")
    # print(f"config_file: {config_file_path}, namespace: {namespace}, loglevel: {loglevel}")

    config = parse_yaml(config_file_path)
    if config is None:
        raise RuntimeError("Failed to load YAML configuration")

    containers = []
    node_containers = config['launch']['node_containers']
    for node_container in node_containers:
        node_container_name = node_container['name']
        print(f"Container Name: {node_container_name}")

        composable_nodes = generate_composable_nodes(config, node_container, namespace, config_file_path)

        container = None
        try:
            container = ComposableNodeContainer(
                name=node_container_name,
                namespace=namespace,
                arguments=['--ros-args', '--log-level', loglevel],
                package='rclcpp_components',
                executable='component_container', # change to component_container_mt to enhance performance
                composable_node_descriptions=composable_nodes,
                output='screen',
            )
        except Exception as e:
            print(f"Error creating ComposableNodeContainer: {e}")
        
        if container is None:
            raise RuntimeError("Failed to create ComposableNodeContainer")

        containers.append(container)

    return LaunchDescription(containers)
