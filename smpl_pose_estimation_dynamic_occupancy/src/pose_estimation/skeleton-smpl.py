#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32MultiArray, String
from zed_msgs.msg import ObjectsStamped
from visualization_msgs.msg import Marker, MarkerArray

class ZEDToSMPLConverter(Node):
    def __init__(self):
        super().__init__('zed_to_smpl_converter')

        self.subscription = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/body_trk/skeletons',
            self.body_tracking_callback,
            10
        )

        self.smpl_publisher = self.create_publisher(
            Float32MultiArray,
            '/smpl_body_pose',
            10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/smpl_markers',
            10
        )
        self.rotation_text_publisher = self.create_publisher(
            String,
            '/smpl_joint_rotations_text',
            10
        )
        self.get_logger().info("ZED 2i to SMPL Converter Node Started")

        self.joint_map = {
            'HEAD': 26, 'NECK': 3, 'SPINE2': 2, 'SPINE1': 1, 'PELVIS': 0,
            'LEFT_CLAVICLE': 4, 'LEFT_SHOULDER': 5, 'LEFT_ELBOW': 6, 'LEFT_WRIST': 7, 'LEFT_HAND': 8,
            'RIGHT_CLAVICLE': 11, 'RIGHT_SHOULDER': 12, 'RIGHT_ELBOW': 13, 'RIGHT_WRIST': 14, 'RIGHT_HAND': 15,
            'LEFT_HIP': 18, 'LEFT_KNEE': 19, 'LEFT_ANKLE': 20, 'LEFT_LEG': 21,
            'RIGHT_HIP': 22, 'RIGHT_KNEE': 23, 'RIGHT_ANKLE': 24, 'RIGHT_LEG': 25,
        }
        self.smpl_order = [
            "PELVIS", "SPINE1", "SPINE2", "NECK", "LEFT_CLAVICLE",
            "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST", "LEFT_HAND",
            "RIGHT_CLAVICLE", "RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST", "RIGHT_HAND",
            "LEFT_HIP", "LEFT_KNEE", "LEFT_ANKLE", "LEFT_LEG",
            "RIGHT_HIP", "RIGHT_KNEE", "RIGHT_ANKLE", "RIGHT_LEG", "HEAD"
        ]
        self.smpl_parents = [
            -1, 0, 1, 2, 3, 4, 5, 6, 7,
            3, 9, 10, 11, 12, 0, 14, 15, 16,
            0, 18, 19, 20, 3
        ]
        self.text_joint_names = [
            "pelvis", "left_hip", "right_hip", "spine1", "left_knee", "right_knee",
            "spine2", "left_ankle", "right_ankle", "spine3", "left_foot", "right_foot",
            "neck", "left_collar", "right_collar", "head", "left_shoulder", "right_shoulder",
            "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_hand"
        ]
        self.prev_pelvis_rotation = None

    def body_tracking_callback(self, msg):
        if len(msg.objects) == 0:
            return

        person = msg.objects[0]
        joints = np.array([kp.kp for kp in person.skeleton_3d.keypoints])
        joints = self.clean_joints(joints)

        smpl_joints = self.convert_to_smpl(joints)
        smpl_rotations = self.compute_smpl_rotations(smpl_joints)
        smpl_shape = self.estimate_shape_parameters(smpl_joints)

        msg_out = Float32MultiArray()
        msg_out.data = np.concatenate([smpl_rotations.flatten(), smpl_shape.flatten()]).tolist()
        self.smpl_publisher.publish(msg_out)
        self.publish_joint_rotations_from_data(smpl_rotations)
        self.publish_markers(smpl_joints)

    def clean_joints(self, joints):
        for i in range(joints.shape[0]):
            if np.isnan(joints[i]).any() or np.all(joints[i] == 0):
                joints[i] = np.array([0.0, 0.0, 0.0])
        return joints

    def convert_to_smpl(self, joints):
        smpl_joints = np.zeros((24, 3))
        for smpl_idx, zed_name in enumerate(self.smpl_order):
            if zed_name in self.joint_map:
                zed_idx = self.joint_map[zed_name]
                smpl_joints[smpl_idx] = joints[zed_idx]
        return smpl_joints

    def compute_smpl_rotations(self, smpl_joints):
        smpl_rotations_global = [None] * 23
        joint_rotvecs = np.zeros((23, 3))
        default_direction = np.array([1.0, 0.0, 0.0])
        for i in range(22):
            parent = self.smpl_parents[i + 1]
            joint_idx = i + 1
            joint_vec = smpl_joints[joint_idx] - smpl_joints[parent]
            norm = np.linalg.norm(joint_vec)
            if norm < 1e-5:
                smpl_rotations_global[i] = R.identity()
                continue
            joint_vec /= norm
            try:
                rel_rot, _ = R.align_vectors([joint_vec], [default_direction])
            except Exception:
                rel_rot = R.identity()
            if parent == 0:
                pelvis_vec = smpl_joints[1] - smpl_joints[0]
                norm_pelvis = np.linalg.norm(pelvis_vec)
                if norm_pelvis < 1e-5:
                    pelvis_rot = R.identity()
                else:
                    pelvis_vec /= norm_pelvis
                    try:
                        pelvis_rot, _ = R.align_vectors([pelvis_vec], [default_direction])
                    except Exception:
                        pelvis_rot = R.identity()
                global_rot = pelvis_rot * rel_rot
            elif parent > 0:
                global_rot = smpl_rotations_global[parent - 1] * rel_rot
            else:
                global_rot = rel_rot
            smpl_rotations_global[i] = global_rot
            joint_rotvecs[i] = global_rot.as_rotvec()

        # ----- Robust pelvis orientation from hip and spine vectors -----
        spine = smpl_joints[1] - smpl_joints[0]
        right_hip = smpl_joints[18]
        left_hip = smpl_joints[14]
        x_axis = right_hip - left_hip
        z_axis = spine

        if np.linalg.norm(x_axis) < 1e-3 or np.linalg.norm(z_axis) < 1e-3:
            pelvis_rot = R.identity()
        else:
            x_axis /= np.linalg.norm(x_axis) + 1e-8
            z_axis /= np.linalg.norm(z_axis) + 1e-8
            y_axis = np.cross(z_axis, x_axis)
            y_axis /= np.linalg.norm(y_axis) + 1e-8
            z_axis = np.cross(x_axis, y_axis)
            z_axis /= np.linalg.norm(z_axis) + 1e-8

            pelvis_matrix = np.column_stack([y_axis, -x_axis, z_axis])

            # Ensure it's a valid rotation matrix
            U, _, Vt = np.linalg.svd(pelvis_matrix)
            rot_matrix = U @ Vt
            if np.linalg.det(rot_matrix) < 0:
                U[:, -1] *= -1
                rot_matrix = U @ Vt

            pelvis_rot = R.from_matrix(rot_matrix)

        # Optional: smoothing (SLERP) to reduce pelvis flicker
        if self.prev_pelvis_rotation is not None:
            from scipy.spatial.transform import Slerp
            key_rots = R.concatenate([self.prev_pelvis_rotation, pelvis_rot])
            times = [0, 1]
            slerp = Slerp(times, key_rots)
            pelvis_rot = slerp(0.5)

        self.prev_pelvis_rotation = pelvis_rot
        pelvis_rotvec = pelvis_rot.as_rotvec()
        joint_rotvecs = np.vstack([pelvis_rotvec, joint_rotvecs])
        return joint_rotvecs

    def estimate_shape_parameters(self, smpl_joints):
        beta = np.zeros(10)
        torso = np.linalg.norm(smpl_joints[3] - smpl_joints[1])
        left_leg = np.linalg.norm(smpl_joints[14] - smpl_joints[15])
        right_leg = np.linalg.norm(smpl_joints[18] - smpl_joints[19])
        avg_leg = (left_leg + right_leg) / 2
        beta[0] = (torso + avg_leg) / 1.7
        return beta

    def publish_markers(self, smpl_joints):
        marker_array = MarkerArray()
        for i, joint in enumerate(smpl_joints):
            marker = Marker()
            marker.header.frame_id = "zed_camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "smpl"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = joint[0]
            marker.pose.position.y = joint[1]
            marker.pose.position.z = joint[2]
            marker.scale.x = 0.04
            marker.scale.y = 0.04
            marker.scale.z = 0.04
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)

    def publish_joint_rotations_from_data(self, rotation_data):
        msg = String()
        rotation_strings = []
        for i in range(min(len(self.text_joint_names), len(rotation_data))):
            rotvec = rotation_data[i]
            rotation_strings.append(
                f"{self.text_joint_names[i]}:[{np.degrees(rotvec[0]):.1f},"
                f"{np.degrees(rotvec[1]):.1f},{np.degrees(rotvec[2]):.1f}]"
            )
        msg.data = " | ".join(rotation_strings)
        self.rotation_text_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZEDToSMPLConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
