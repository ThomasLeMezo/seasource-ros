import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import socket
from enum import Enum, IntEnum

def generate_launch_description():
    home_path = os.path.expanduser('~')
    hostname = socket.gethostname()
    parameters_file_list = []

    config_seasource = os.path.join(
        home_path,
        'config/default/',  # Directory where yaml are
        'seasource.yaml'  # Name of the file
    )
    if os.path.exists(config_seasource):
        parameters_file_list.append(config_seasource)

    # Node list
    list_node = []

    gpsd_node = Node(
        package='gpsd_client',
        executable='gpsd_node',
        namespace='driver',
        name='gpsd_node',
        respawn=True,
        parameters=parameters_file_list
    )
    list_node.append(gpsd_node)

    xbee_node = Node(
        package='xbee_driver',
        executable='xbee',
        namespace='driver',
        name='xbee_node',
        respawn=False,
        parameters=parameters_file_list
    )
    list_node.append(xbee_node)

    seasource_audio_node = Node(
        package='seasource_audio',
        executable='seasource_audio_node',
        namespace='',
        name='seasource_audio_node',
        respawn=True,
        parameters=parameters_file_list
    )
    list_node.append(seasource_audio_node)

    bag_recorder = Node(
        package='seasource_recorder_cpp',
        executable='recorder',
        namespace='observer',
        output='screen',
        name='recorder',
        respawn=True,
        parameters=parameters_file_list
    )

    return LaunchDescription(list_node)
