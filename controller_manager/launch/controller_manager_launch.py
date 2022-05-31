from os.path import join

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import yaml
import logging


def generate_launch_description():
    config = join(
        get_package_share_directory('controller_manager'),
        'config',
        'controller_manager.yaml'
    )

    with open(config, "r") as f:
        config_params = yaml.safe_load(f)

    try:
        plugin_name = config_params["/**"]["ros__parameters"]["plugin_name"]
    except KeyError:
        plugin_name = ""

    if not plugin_name:
        logging.error("Plugin not set.")
        exit(-1)

    try:
        plugin_config = config_params["/**"]["ros__parameters"]["plugin_config_file"]
    except KeyError:
        plugin_config = ""

    if not plugin_config:
        plugin_config = join(
            get_package_share_directory(plugin_name),
            'config',
            'default_controller.yaml'
        )

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        Node(
            package='controller_manager',
            executable='controller_manager_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[config, plugin_config],
            output='screen',
            emulate_tty=True
        )
    ])
