# Zed-Ouster-based-Real-time-SMPL-Pose-and-Dynamic-Occupancy-Estimation
# Sensors Used

Zed 2i Camera

Ouster OS0 LiDAR

# Pre-requisites

ROS2 Humble: https://docs.ros.org/en/humble/index.html

ZED 2i SDK: https://www.stereolabs.com/en-de/developers/release

ZED ROS2 Wrapper: https://github.com/stereolabs/zed-ros2-wrapper

Ouster ROS2 Repository: https://github.com/ouster-lidar/ouster-ros

# Steps for SMPL based Pose Estimation
Navigate to ZED workspace

Launch ZED 2i Camera
```bash
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zed2i
```

Install the Repository in your Workspace
```bash
git clone https://github.com/swarajtendulkar10/Zed-Ouster-based-Real-time-SMPL-Pose-and-Dynamic-Occupancy-Estimation.git
```
Add Folders in the Root Directory
```bash
mkdir launch/ worlds/ config/ include/ description/
```

Build the Workspace
```bash
colcon build
```

Launch the Skeleton Python File
```bash
ros2 run smpl_pose_estimation_dynamic_occupancy skeleton-smpl.py 
```

Launch the Visualizer
```bash
ros2 run smpl_pose_estimation_dynamic_occupancy visualizer.py 
```
# Steps for LiDAR based Dynamic Occupancy Estimation

Navigate to Ouster workspace

Launch Ouster LiDAR
```bash
roslaunch ouster_ros driver.launch      \
    sensor_hostname:=<sensor host name> \
 ```   
Close all the RVIZ windows and relaunch RVIZ
```bash
rviz2
```
Open config and load the RVIZ file in the repository

Fuse Camera and LiDAR
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 os_sensor zed_camera_link
```
# Visualize the fused data

Set the parent link to os_sensor
Add the topics /smpl_markers and /smpl_joint_axes


Run the Segmentation Node
```bash
ros2 run smpl_pose_estimation_dynamic_occupancy segment.py
```
Add the segmented points topic in RVIZ 

Run the Dynamic Occupancy Grid Node
```bash
ros2 run smpl_pose_estimation_dynamic_occupancy dogma.py
```
Add the dynamic occupancy grid topic in RVIZ
# Implementation Results
<img width="600" height="600" alt="Segmented_Dynamic_Occupancy" src="https://github.com/user-attachments/assets/adba846f-c969-4a8b-bb20-728198519e5f" />
