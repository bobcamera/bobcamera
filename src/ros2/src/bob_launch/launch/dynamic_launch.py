import os
import yaml
import rclpy
import copy
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable 
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState

def parse_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading YAML file: {e}")
        return None
    

def get_node_parameters(config, node_name):
    return config['basic_config'].get(node_name, {}) | config['advanced_config'].get(node_name, {})


def create_lifecycle_client(node_name, transition_id):
    rclpy.init()
    node = rclpy.create_node('lifecycle_manager')
    client = node.create_client(ChangeState, f'/{node_name}/change_state')
    req = ChangeState.Request()
    req.transition.id = transition_id

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f'Waiting for {node_name} change_state service...')

    future = client.call_async(req)

    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(f'Successfully changed state of {node_name}')
    else:
        node.get_logger().error(f'Failed to change state of {node_name}')

    rclpy.shutdown()


def create_lifecycle_manager(node_names, transition_id):
    rclpy.init()
    print(node_names)

    for node_name in node_names:
        node = rclpy.create_node('lifecycle_manager')
        client = node.create_client(ChangeState, f'/{node_name}/change_state')
        req = ChangeState.Request()
        req.transition.id = transition_id
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f'Waiting for {node_name} change_state service...')

        future = client.call_async(req)

        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            node.get_logger().info(f'Successfully changed state of {node_name}')
        else:
            node.get_logger().error(f'Failed to change state of {node_name}')

    rclpy.shutdown()


def generate_composable_nodes(config, node_container, namespace):
    node_descriptions = node_container.get('nodes', [])
    if node_descriptions is None:
        return []

    nodes = []
    for node_description in node_descriptions:
        remappings = [
            (k, v) for mapping in node_description.get('remappings', [])
            for k, v in mapping.items()
        ]
        parameters = get_node_parameters(config, node_description['name'])

        print(f"  Node: Package: {node_description['package']}, Plugin: {node_description['plugin']}, Name: {node_description['name']}")
        node = ComposableNode(
            package=node_description['package'],
            plugin=node_description['plugin'],
            name=node_description['name'],
            namespace=namespace,
            parameters=[parameters],
            extra_arguments=[{'use_intra_process_comms': True}],
            remappings=remappings
        )
        nodes.append(node)

    return nodes


def generate_containers(config, namespace, loglevel):
    node_containers = config['launch'].get('node_containers', [])
    if node_containers is None:
        return []

    containers = []
    for node_container in node_containers:
        node_container_name = node_container['name']
        node_container_executable = node_container.get('executable', 'component_container')
        node_container_output = node_container.get('output', 'screen')
        print(f"Container Name: {node_container_name}, executable: {node_container_executable}")

        composable_nodes = generate_composable_nodes(config, node_container, namespace)

        container = None
        try:
            container = ComposableNodeContainer(
                name=node_container_name,
                namespace=namespace,
                arguments=['--ros-args', '--log-level', loglevel],
                package='rclcpp_components',
                executable=node_container_executable,
                composable_node_descriptions=composable_nodes,
                output=node_container_output,
            )
        except Exception as e:
            print(f"Error creating ComposableNodeContainer: {e}")
        
        if container is None:
            raise RuntimeError("Failed to create ComposableNodeContainer")

        containers.append(container)
            
    return containers


def generate_standalone_nodes(config, namespace, loglevel):
    config_nodes = config['launch'].get('standalone_nodes', [])
    if config_nodes is None:
        return []

    nodes = []
    for config_node in config_nodes:
        remappings = [
            (k, v) for mapping in config_node.get('remappings', [])
            for k, v in mapping.items()
        ]
        parameters = get_node_parameters(config, config_node['name'])
        print(f"Standalone Node: Package: {config_node['package']}, Executable: {config_node['executable']}, Name: {config_node['name']}")
        print(f"  Parameters: {parameters}")

        node = None
        try:
            node = Node(
                package=config_node['package'],
                namespace=namespace,
                executable=config_node['executable'],
                name=config_node['name'],
                arguments=['--ros-args', '--log-level', loglevel],
                parameters=[parameters],
                remappings=remappings,
                output=config_node.get('output', 'screen')
            )
        except Exception as e:
            print(f"Error creating Standalone Node: {e}")
        
        if node is None:
            raise RuntimeError("Failed to create Standalone Node")

        nodes.append(node)
    
    return nodes


def generate_launch_description():
    config_file_path = 'assets/config/app_config.yaml'

    config = parse_yaml(config_file_path)
    if config is None:
        raise RuntimeError("Failed to load YAML configuration")

    loglevel = EnvironmentVariable('BOB_LOGLEVEL', default_value="INFO")
    namespace = config['launch'].get('namespace', '')
    rosbridge_enable = config['launch'].get('rosbridge', True)

    launch_list = generate_containers(config, namespace, loglevel)
    launch_list.extend(generate_standalone_nodes(config, namespace, loglevel))

    if (rosbridge_enable):
        ros_bridge_package_xml = os.path.join(get_package_share_directory('rosbridge_server'), 'launch/rosbridge_websocket_launch.xml')
        rosbridge_launch = IncludeLaunchDescription(FrontendLaunchDescriptionSource(ros_bridge_package_xml))
        launch_list.append(rosbridge_launch)

    return LaunchDescription(launch_list)
