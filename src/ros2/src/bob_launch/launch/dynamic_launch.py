import yaml
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable 

def parse_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading YAML file: {e}")
        return None
    

def get_node_parameters(config, node_name):
    parameters = config['basic_config'].get(node_name, {}) | config['advanced_config'].get(node_name, {})
    return parameters


def generate_composable_nodes(config, node_container, namespace):
    node_descriptions = node_container['nodes']
    nodes = []
    for node_description in node_descriptions:
        remappings = [
            (k, v) for mapping in node_description.get('remappings', [])
            for k, v in mapping.items()
        ]
        parameters = get_node_parameters(config, node_description['name'])

        print(f"Node: Package: {node_description['package']}, Plugin: {node_description['plugin']}, Name: {node_description['name']}")
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
    containers = []
    node_containers = config['launch']['node_containers']
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
                executable=node_container_executable, # change to component_container_mt to enhance performance
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
    nodes = []
    config_nodes = config['launch'].get('standalone_nodes', [])
    for config_node in config_nodes:
        remappings = [
            (k, v) for mapping in config_node.get('remappings', [])
            for k, v in mapping.items()
        ]
        parameters = get_node_parameters(config, config_node['name'])
        print(f"Standalone Node: Package: {config_node['package']}, Executable: {config_node['executable']}, Name: {config_node['name']}")

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
    namespace = EnvironmentVariable('BOB_NAMESPACE', default_value="")
    loglevel = EnvironmentVariable('BOB_LOGLEVEL', default_value="INFO")

    config = parse_yaml(config_file_path)
    if config is None:
        raise RuntimeError("Failed to load YAML configuration")

    containers = generate_containers(config, namespace, loglevel)
    containers.extend(generate_standalone_nodes(config, namespace, loglevel))

    return LaunchDescription(containers)
