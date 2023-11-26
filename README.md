## Land Vehicle Obstacle Avoidance
This repository contains the source code for a ROS2 package that provides an implementation of a simple land vehicle obstacle avoidance system using the sensors

### System Requirements 
- **OS:** [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)
- **Simulation Program:** [Gazebo Garden](https://gazebosim.org/docs/garden/getstarted)
- **ROS2:** [Humble Hawksbill](https://docs.ros.org/en/humble/index.html)

### Install
```
cd ~/
git clone https://github.com/bilgeemert/obstacle_avoidance
```

### Compile
- The path to the model should be added to the **GZ_SIM_RESOURCE_PAT** environment variable
```
echo "export GZ_VERSION=garden" >> ~/.bashrc

echo "export GZ_SIM_RESOURCE_PATH=/home/${USER}/obstacle_avoidance/src/obstacle_avoidance/models" >> ~/.bashrc 

source ~/.bashrc  #or bash or simply restart the terminal
```
- When using the 'bash' command to reload the terminal, it restarts the terminal session, while 'source ~/.bashrc' reloads the .bashrc file, applying the changes for the current session

```
source /opt/ros/humble/setup.bash

cd ~/obstacle_avoidance

#Builds all files in the project.
colcon build  

#Or only build a specific package.
colcon build --packages-select [package_name]
```

### Run

```
source /opt/ros/humble/setup.bash
source ~/obstacle_avoidance/install/setup.bash
```
- To avoid entering the **`~/obstacle_avoidance/install/setup.bash`** command every time a new terminal is opened, these commands are written to the **`~/.bashrc`** file.
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /home/${USER}/obstacle_avoidance/install/setup.bash" >> ~/.bashrc
```
- The control method is specified in the 'control_unit' variable in the params file to control the vehicle. By default, it is set to keyboard control. If desired, it can also be controlled using a joystick or an ESP8266.
1. **Keyboard controls are as follows:**

    - 'w' for forward            
    - 's' to stop                                                   
    - 'x' for backward
    - 'a' for left
    - 'd' for right
```
    w
a   s    d
    x
```
2. **Joystick control:** Control is achieved using the left joystick.
3. **ESP8266 control:** Joy data is sent to the computer's serial port via an interface prepared using [RemoteXY](https://remotexy.com/en/editor/).
    * To connect to the ESP8266 board, it is necessary to write the port to which the board is connected in the `file_name` variable in the params file. By default, it is `/dev/ttyUSB0`
    * The RemoteXY application is downloaded to the phone, and to connect to the access point broadcasted by ESP8266, you connect to `Obstacle_Avoidance` in the Wi-Fi section. The password is `obstacle_avoidance`.
- Instead of running each code individually, the launch file is executed.
```
ros2 launch  obstacle_avoidance  land_launch.py
```
- If files are to be launched individually, the path to the params file should be provided during the launch.
```
ros2 run controller controller_node --ros-args --params-file /home/${USER}/obstacle_avoidance/src/obstacle_avoidance/config/params.yaml
```

### Project
The project aims to create a semi-autonomous ground vehicle capable of navigating its environment and avoiding obstacles. The robot will use sensors such as Lidar. In the later stages, it will maintain its direction with the help of an IMU (Inertial Measurement Unit).

- **Gazebo Garden:** Simulation settings can be modified by assigning the topic name `lidar` for the `lidar` window to display Lidar data. If it is desired to hide areas where detection is not performed, unchecking the `show non-hitting rays` option is recommended

![Gazebo Garden](image/gazebo.png)

- **Rviz2:** The Lidar sensor data is visualized in conjunction with the camera at the front of the vehicle, as well as the data from the joystick and the error information from obstacle avoidance. 
    * The **RED** arrow represents the linear velocity.
    * The **GREEN** arrow represents the angular velocity.
    * The **BLUE** arrow represents the result vector.

![Rviz](image/rviz.png)

### Result 
The vehicle is initialized in a specific environment. The user provides only the linear velocity command. In most cases, the vehicle successfully avoids obstacles by providing angular velocity. In some situations, when the vehicle gets too close to obstacles (enters the safety zone), the linear velocity command is reset, and the vehicle is directed to turn in a specific direction by examining right and left sensor data to avoid the obstacle. (The video is in 2x playback speed.)



https://github.com/bilgeemert/obstacle_avoidance/assets/79978810/49e35912-d6f7-4025-8f00-1103b33972f5


