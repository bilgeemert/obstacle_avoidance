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

land_vehicle_path     = get_package_share_directory("obstacle_avoidance")
# simulation_world_path = Path(land_vehicle_path, "world", "land_vehicle.sdf") land_tunnel.sdf
simulation_world_path = Path(land_vehicle_path, "world", "land_tunnel.sdf") 
simulation_model_path = Path(land_vehicle_path, "models")

config_file = os.path.join(
    get_package_share_directory('obstacle_avoidance'),
    'config',
    'params.yaml'
  )

simulation = ExecuteProcess(
    cmd=["gz", "sim", "-r", simulation_world_path]
  )

rviz = ExecuteProcess(
    cmd=["rviz2"]
  )

serial_node = Node(
            package="command",                                               
            executable="command_node",
            parameters=[config_file],
            output="screen"
          )

bridge_control = Node(
            package="ros_gz_bridge",                                               
            executable="parameter_bridge",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
            ],
            output="screen"
          )

bridge_keyboard = Node(
            package="ros_gz_bridge",                                               
            executable="parameter_bridge",
            arguments=[
                "/keyboard/keypress@std_msgs/msg/Int32[gz.msgs.Int32"
            ],
            remappings=[("/keyboard/keypress","/keypress")],
            output="screen"
          )

bridge_lidar = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
            ],
            remappings=[("/lidar/points","/lidar")],
          )

bridge_camera = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/camera@sensor_msgs/msg/Image[gz.msgs.Image"
            ]
          )

bridge_imu = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
            ]
          )

controller = Node(
            package="controller",                                               
            executable="controller_node",
            output="screen"
          )

shutdown = RegisterEventHandler(
            event_handler=OnProcessExit(
              target_action=simulation,
              on_exit=[EmitEvent(event=Shutdown)]
            )
          )

joy_node = Node(
            package="joy",                                               # ros_ign_bridge eski versiyonda kullanılır.
            executable="joy_node",
            parameters=[config_file],
            output="screen"
          )

def generate_launch_description():
    return LaunchDescription([
        serial_node,
        simulation,
        joy_node,
        # controller,
        # rviz,
        
        bridge_keyboard,
        bridge_control,
        bridge_lidar,
        bridge_camera,
        # bridge_imu,

        shutdown
    ]) 
