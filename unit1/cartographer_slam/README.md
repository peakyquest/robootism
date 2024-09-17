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

# Cartographer Configuration Guide

This guide provides instructions on configuring Cartographer for different robots and sensors to achieve optimal mapping results. Cartographer uses a Lua configuration file to set parameters for SLAM (Simultaneous Localization and Mapping).

## 1. Cartographer Subscribed Topics

Cartographer automatically subscribes to the following topics:
- `/scan`: Laser scan data (`sensor_msgs/LaserScan`).
- `/odom`: Odometry data (`nav_msgs/Odometry`).
- `/imu`: IMU data (`sensor_msgs/Imu`).

**Note**: These topic names cannot be changed directly. If your robot uses different topic names, remap them to match Cartographer’s expected topics.

---

## 2. Configurable Parameters in the Lua File

### General Parameters

1. **`map_frame`**:  
   The ROS frame ID for publishing sub-maps. Typically set to `map`.
   
2. **`tracking_frame`**:  
   The frame tracked by SLAM. If using an IMU, it should correspond to the IMU position. Common choices include `base_link` or `base_footprint`.

3. **`published_frame`**:  
   Frame for publishing poses. If odometry is supplied, use `odom`. Otherwise, use `base_link`.

4. **`odom_frame`**:  
   Used if `provide_odom_frame` is enabled. Represents the non-loop-closed local SLAM result. Typically set to `odom`.

5. **`provide_odom_frame`**:  
   Set to `true` to publish the continuous pose as `odom_frame` in the `map_frame`.

6. **`use_odometry`**:  
   Set to `true` to use odometry data. Must subscribe to `/odom`.

7. **`use_nav_sat`**:  
   Set to `true` to use GPS data. Must subscribe to `/fix`.

---

### Laser Parameters

1. **`num_laser_scans`**:  
   Number of laser scan topics. Typically set to `1` for one scanner (`/scan`), and `n` for multiple scanners (`/scan_1`, `/scan_2`, etc.).

2. **`num_multi_echo_laser_scans`**:  
   Number of multi-echo laser scan topics, if using multi-echo laser scanners.

3. **`num_subdivisions_per_laser_scan`**:  
   Splits each laser scan into multiple point clouds to compensate for scanner motion. Adjust this parameter based on your robot's speed.

4. **`num_point_clouds`**:  
   Number of point cloud topics, usually for 3D LiDAR (`/points2`, `/points2_1`, `/points2_2`, etc.).

---

### Filter Parameters

1. **`lookup_transform_timeout_sec`**:  
   Time to wait for transforms, in seconds. Default is 0.3 seconds.

2. **`submap_publish_period_sec`**:  
   Time interval to publish submap poses, e.g., `0.3` seconds.

3. **`pose_publish_period_sec`**:  
   Time interval to publish robot poses, e.g., `5e-3` for 200 Hz.

4. **`trajectory_publish_period_sec`**:  
   Time interval to publish trajectory markers, e.g., `30e-3` for 30 ms.

5. **Sampling Ratios**:
   - **`odometry_sampling_ratio`**: Ratio for odometry message sampling.
   - **`fixed_frame_sampling_ratio`**: Ratio for fixed frame message sampling.
   - **`imu_sampling_ratio`**: Ratio for IMU message sampling.
   - **`landmarks_sampling_ratio`**: Ratio for landmark message sampling.

---

### Trajectory Builder Parameters

1. **`TRAJECTORY_BUILDER_2D.min_range`**:  
   Minimum range (in meters) to consider for SLAM.

2. **`TRAJECTORY_BUILDER_2D.max_range`**:  
   Maximum range (in meters) to consider for SLAM.

3. **`TRAJECTORY_BUILDER_2D.missing_data_ray_length`**:  
   Length (in meters) for missing data rays.

4. **`TRAJECTORY_BUILDER_2D.use_imu_data`**:  
   Set to `true` if IMU data is available and should be used.

---

## Saving a Map
To save the map for later use, simply write the following command in the command line 

```
ros2 run nav2_map_server map_saver_cli -f <file_name>
```
