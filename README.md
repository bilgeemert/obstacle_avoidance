## Land Vehicle Obstacle Avoidance
This repository contains the source code for a ROS2 package that provides an implementation of a simple land vehicle obstacle avoidance system using the sensor

### System Requirements 
- **OS:** [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)
- **Simulation Program:** [Gazebo Garden](https://gazebosim.org/docs/garden/getstarted)
- **ROS2:** [Humble Hawksbill](https://docs.ros.org/en/humble/index.html)

### Install
```
mkdir ~/obstacle_avoidance
cd ~/obstacle_avoidance
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
cd ~/obstacle_avoidance

#Builds all files in the project.
colcon build  

#Or only build a specific package.
colcon build --packages-select [package_name]
```

### Run
```
source install/local_setup.bash
```