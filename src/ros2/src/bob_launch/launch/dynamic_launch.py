import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable 
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import rclpy.logging

logger = rclpy.logging.get_logger('dynamic_launch')

def parse_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            content = file.read()
            content = content.replace('\t', '  ')
            return yaml.safe_load(content)
    except Exception as e:
        logger.error(f"Error loading YAML file: {e}")
        return None


def subst_variables(config, params):
    for key, value in params.items():
        if isinstance(value, str) and value.startswith('$'):
            params[key] = config['variables'].get(value[1:])
    

def get_node_parameters(config, node_name):
    params = config['basic_config'].get(node_name, {}) | config['advanced_config'].get(node_name, {})
    if 'copy' in params:
        node_copy = params.pop('copy')
        params = get_node_parameters(config, node_copy) | params
    subst_variables(config, params)
    return params


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
        if parameters.get('enable', True):
            logger.info(f"{TextStyle.BRIGHT_CYAN}  Node: Name: {TextStyle.BOLD+TextStyle.BRIGHT_WHITE}{node_description['name']}{TextStyle.RESET+TextStyle.BRIGHT_CYAN}, Package: {TextStyle.RESET}{node_description['package']}{TextStyle.BRIGHT_CYAN}, Plugin: {TextStyle.RESET}{node_description['plugin']}")
            logger.debug(f"  Parameters: {parameters}")
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
        subst_variables(config, node_container)
        node_container_name = node_container['name']
        node_container_executable = node_container.get('executable', 'component_container')
        node_container_output = node_container.get('output', 'screen')
        logger.info(f"{TextStyle.YELLOW}Container Name: {TextStyle.BOLD+TextStyle.BRIGHT_WHITE}{node_container_name}{TextStyle.RESET+TextStyle.YELLOW}, executable: {TextStyle.RESET}{node_container_executable}")

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
            logger.error(f"Error creating ComposableNodeContainer: {e}")
        
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
        subst_variables(config, config_node)
        parameters = get_node_parameters(config, config_node['name'])
        logger.info(f"{TextStyle.GREEN}Standalone Node:{TextStyle.BRIGHT_GREEN} Name:{TextStyle.RESET+TextStyle.BOLD+TextStyle.BRIGHT_WHITE} {config_node['name']}{TextStyle.RESET+TextStyle.BRIGHT_GREEN}, Package: {TextStyle.RESET}{config_node['package']}{TextStyle.BRIGHT_GREEN}, Executable: {TextStyle.RESET}{config_node['executable']}")
        logger.debug(f"  Parameters: {parameters}")

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
            logger.error(f"Error creating Standalone Node: {e}")
        
        if node is None:
            raise RuntimeError("Failed to create Standalone Node")

        nodes.append(node)
    
    return nodes


def generate_lifecycle_manager(config, namespace, loglevel):
    parameters = get_node_parameters(config, 'lifecycle_manager_node')
    return ComposableNodeContainer(
                name='LifeCycleManager',
                namespace=namespace,
                arguments=['--ros-args', '--log-level', loglevel],
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[ComposableNode(
                                                package='bob_auxiliary',
                                                plugin='LifecycleManager',
                                                name='lifecycle_manager_node',
                                                namespace=namespace,
                                                parameters=[parameters],
                                                extra_arguments=[{'use_intra_process_comms': True}],
                                                remappings=[]
                                            )],
                output='both',
            )


def generate_launch_description():
    config_file_path = 'assets/config/app_config.yaml'

    config = parse_yaml(config_file_path)
    if config is None:
        raise RuntimeError("Failed to load YAML configuration")

    logger.info(f"{TextStyle.BG_GREEN+TextStyle.BOLD+TextStyle.BLACK}    Adding Containers and Stand-alone nodes to Launch    {TextStyle.RESET}")

    loglevel = EnvironmentVariable('BOB_LOGLEVEL', default_value="INFO")
    namespace = config['launch'].get('namespace', '')
    rosbridge_enable = config['launch'].get('rosbridge', True)
    lifecycle_enable = config['launch'].get('lifecycle', True)

    launch_list = generate_containers(config, namespace, loglevel)
    launch_list.extend(generate_standalone_nodes(config, namespace, loglevel))

    if (rosbridge_enable):
        logger.info(f"{TextStyle.BRIGHT_MAGENTA}Package: {TextStyle.RESET+TextStyle.BOLD+TextStyle.BRIGHT_WHITE}RosBridge{TextStyle.RESET}")
        ros_bridge_package_xml = os.path.join(get_package_share_directory('rosbridge_server'), 'launch/rosbridge_websocket_launch.xml')
        launch_list.append(IncludeLaunchDescription(FrontendLaunchDescriptionSource(ros_bridge_package_xml)))

    if (lifecycle_enable):
        logger.info(f"{TextStyle.BRIGHT_MAGENTA}Package: {TextStyle.RESET+TextStyle.BOLD+TextStyle.BRIGHT_WHITE}LifeCycle{TextStyle.RESET}")
        launch_list.append(generate_lifecycle_manager(config, namespace, loglevel))

    logger.info(f"{TextStyle.BG_GREEN+TextStyle.BOLD+TextStyle.BLACK}    All Added    {TextStyle.RESET}")

    return LaunchDescription(launch_list)


class TextStyle:
    # Foreground (Text) Colors
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'

    # Bright Foreground (Text) Colors
    BRIGHT_BLACK = '\033[90m'
    BRIGHT_RED = '\033[91m'
    BRIGHT_GREEN = '\033[92m'
    BRIGHT_YELLOW = '\033[93m'
    BRIGHT_BLUE = '\033[94m'
    BRIGHT_MAGENTA = '\033[95m'
    BRIGHT_CYAN = '\033[96m'
    BRIGHT_WHITE = '\033[97m'

    # Background Colors
    BG_BLACK = '\033[40m'
    BG_RED = '\033[41m'
    BG_GREEN = '\033[42m'
    BG_YELLOW = '\033[43m'
    BG_BLUE = '\033[44m'
    BG_MAGENTA = '\033[45m'
    BG_CYAN = '\033[46m'
    BG_WHITE = '\033[47m'

    # Bright Background Colors
    BG_BRIGHT_BLACK = '\033[100m'
    BG_BRIGHT_RED = '\033[101m'
    BG_BRIGHT_GREEN = '\033[102m'
    BG_BRIGHT_YELLOW = '\033[103m'
    BG_BRIGHT_BLUE = '\033[104m'
    BG_BRIGHT_MAGENTA = '\033[105m'
    BG_BRIGHT_CYAN = '\033[106m'
    BG_BRIGHT_WHITE = '\033[107m'

    # Styles
    RESET = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'