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

Launch ZED 2i Camera


Install the Repository in your Workspace

Add Folders in the Root Directory

Build the Workspace
```bash
colcon build
```
Launch the Skeleton Python File

Launch the Visualizer

# Steps for LiDAR based Dynamic Occupancy Estimation
Launch Ouster LiDAR

Install the Repository in your Workspace

Add Folders in the Root Directory

Build the Workspace
```bash
colcon build
```
Fuse Camera and LiDAR

Visualize the fused data

RVIZ Configuration 

Run the Segmentation Node




Run the Dynamic Occupancy Grid Node


