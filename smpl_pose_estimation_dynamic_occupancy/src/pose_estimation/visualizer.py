#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation as R, Slerp

class JointAxisVisualizer(Node):
    def __init__(self):
        super().__init__('joint_axis_visualizer')

        self.n_joints = 24  # SMPL has 24 joints
        self.joint_positions = np.zeros((self.n_joints, 3))
        self.smoothed_positions = np.zeros_like(self.joint_positions)
        self.received_positions = False
        self.smoothing_factor = 0.8  # 0 = no smoothing, 1 = max smoothing
        self.initialized = False
        self.prev_rotvecs = [None] * self.n_joints

        self.sub_markers = self.create_subscription(
            MarkerArray, '/smpl_markers', self.markers_callback, 10)
        self.sub_pose = self.create_subscription(
            Float32MultiArray, '/smpl_body_pose', self.pose_callback, 10)
        self.axis_publisher = self.create_publisher(
            MarkerArray, '/smpl_joint_axes', 10)
        self.get_logger().info("Joint Axis Visualizer started.")

        # Joints to show: Pelvis, Spine1, Spine2, Neck, Left Clavicle, Right Clavicle, Left Hip, Right Hip
        self.show_indices = [0, 1, 2, 3, 4, 9, 14, 18]

    def markers_callback(self, msg):
        for marker in msg.markers:
            self.joint_positions[marker.id] = [
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ]
        # Smooth positions
        if not self.initialized:
            self.smoothed_positions[:] = self.joint_positions
            self.initialized = True
        else:
            self.smoothed_positions = (
                self.smoothing_factor * self.smoothed_positions +
                (1 - self.smoothing_factor) * self.joint_positions
            )
        self.received_positions = True

    def pose_callback(self, msg):
        if not self.received_positions:
            return
        n_joints = min(len(self.smoothed_positions), len(msg.data)//3)
        rotations = np.array(msg.data[:n_joints*3]).reshape((n_joints, 3))
        marker_array = MarkerArray()
        axis_len = 0.1  # 10cm
        min_axis_len = 1e-3

        for i in self.show_indices:
            if i >= n_joints:
                continue
            pos = self.smoothed_positions[i]
            rotvec = rotations[i]
            # SLERP smoothing for orientation
            if self.prev_rotvecs[i] is not None and not np.allclose(self.prev_rotvecs[i], [0,0,0]):
                prev_rot = R.from_rotvec(self.prev_rotvecs[i])
                curr_rot = R.from_rotvec(rotvec)
                slerp = Slerp([0,1], R.concatenate([prev_rot, curr_rot]))
                try:
                    rot = slerp(0.5)
                    rotvec = rot.as_rotvec()
                except Exception:
                    rot = curr_rot
            else:
                rot = R.from_rotvec(rotvec)
            self.prev_rotvecs[i] = rotvec.copy()
            rot_matrix = rot.as_matrix()
            if np.any(np.linalg.norm(rot_matrix, axis=0) < min_axis_len):
                continue
            colors = [
                (1.0, 0.0, 0.0),  # X - red
                (0.0, 1.0, 0.0),  # Y - green
                (0.0, 0.0, 1.0),  # Z - blue
            ]
            for j in range(3):
                dir_vec = rot_matrix[:, j] * axis_len
                endpoint = pos + dir_vec
                marker = Marker()
                marker.header.frame_id = "zed_camera_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"axis_{j}"
                marker.id = i*3 + j
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.points = [
                    Point(x=pos[0], y=pos[1], z=pos[2]),
                    Point(x=endpoint[0], y=endpoint[1], z=endpoint[2]),
                ]
                marker.scale.x = 0.005
                marker.scale.y = 0.015
                marker.scale.z = 0.015
                marker.color.r = colors[j][0]
                marker.color.g = colors[j][1]
                marker.color.b = colors[j][2]
                marker.color.a = 1.0
                marker_array.markers.append(marker)
        self.axis_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = JointAxisVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
