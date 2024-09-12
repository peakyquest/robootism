# Robotisim 



## Task 1: Lidar-Based Map Creation into Occupancy Grid Map 

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
```



7. Place the objects in the empty world, so the lidar can detect the object and creat map easily. 

![default_gzclient_camera(1)-2024-09-12T23_57_40 282092](https://github.com/user-attachments/assets/0959d0e5-0fa4-48da-87fb-6ba391511c02)


8. Launch the cartographer.launch.py to create map

```
cd ~/ros2_ws
colcon build --packages-select cartographer_slam
source ~/ros2_ws/install/setup.bash
ros2 launch cartographer_slam cartographer.launch.py
```

9. Open Rviz
```
rviz2 
```
Click the Add button under Displays and choose the Map display.

![button](https://github.com/user-attachments/assets/3a23f474-b9d5-4712-b79d-16961ce195cf)

![add](https://github.com/user-attachments/assets/bf962660-f879-4abd-a0bd-970cbe6f3fff)

![Screenshot from 2024-09-12 23-59-56](https://github.com/user-attachments/assets/2a375566-e0e6-41cb-8f40-11fbba4ee7a7)






