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

land_vehicle_path     = get_package_share_directory("land_vehicle")
simulation_world_path = Path(land_vehicle_path, "world", "land_vehicle.sdf")
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
            package="serial_comm",                                               # ros_ign_bridge eski versiyonda kullanılır.
            executable="serial_comm_node",
            parameters=[config_file],
            output="screen"
          )

bridge_control = Node(
            package="ros_gz_bridge",                                               # ros_ign_bridge eski versiyonda kullanılır.
            executable="parameter_bridge",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
            ],
            output="screen"
          )


bridge_lidar = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                # "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"  # Sadece bir ekseni veriyor
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
            package="controller",                                               # ros_ign_bridge eski versiyonda kullanılır.
            executable="controller_node",
            output="screen"
          )

shutdown = RegisterEventHandler(
            event_handler=OnProcessExit(
              target_action=simulation,
              on_exit=[EmitEvent(event=Shutdown)]
            )
          )

def generate_launch_description():
    return LaunchDescription([
        serial_node,
        simulation,
        # controller,
        # rviz,

        bridge_control,
        bridge_lidar,
        bridge_camera,
        bridge_imu,

        shutdown
    ]) 
