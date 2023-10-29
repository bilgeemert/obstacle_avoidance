import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os.path import join as Path

# Similasyon kapatılınca programda sonlanması sağlanır.
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

config_file = os.path.join(
    get_package_share_directory('config_file'),
    'config',
    'params.yaml'
  )

serial_node = Node(
            package="serial_comm",                                               # ros_ign_bridge eski versiyonda kullanılır.
            executable="serial_comm_node",
            parameters=[config_file],
            output="screen"
          )

def generate_launch_description():
    return LaunchDescription([
        serial_node       
    ]) 
