# Robotisim 



## Task 1: Install Gazebo, Cartographer, and Navigation2

1. Install Gazebo

Gazebo is a powerful simulation tool used for testing and developing robotics applications. To install Gazebo, if you haven't installed yet:

- Open your terminal.
- Execute the following command:

```
  sudo apt install ros-humble-gazebo-*
```
This command will install Gazebo and all necessary Gazebo-related packages for ROS 2 Humble.

2. Install Cartographer

Cartographer is a library for real-time simultaneous localization and mapping (SLAM). To install Cartographer: In your terminal, execute the following commands:

```
  sudo apt install ros-humble-cartographer
  sudo apt install ros-humble-cartographer-ros
```
These commands will install Cartographer and the ROS 2 integration packages for Cartographer.

3. Install Navigation2

Navigation2 provides advanced capabilities for autonomous robot navigation. To install Navigation2: Execute the following commands in your terminal:

  
```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```
These commands will install Navigation2 and the bringup package to get you started with Navigation2.

4. Install Turlebot3 Packages

```
sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3

echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc
```

5. Install Turtlebot3 Simulation 

```
cd ~/turtlebot3_ws/src/
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws && colcon build --symlink-install
```

6. Launch the Turtlebot3 Simulation
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
