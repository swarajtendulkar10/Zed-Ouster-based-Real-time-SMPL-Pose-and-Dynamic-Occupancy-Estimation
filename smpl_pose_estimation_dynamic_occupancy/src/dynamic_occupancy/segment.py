#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from zed_interfaces.msg import ObjectsStamped
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class OusterBridgeNode(Node):
    def __init__(self):
        super().__init__('ouster_bridge_node')

        # Subscriber to the Ouster points topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.listener_callback_pointcloud,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher to the new point cloud topic
        self.publisher = self.create_publisher(PointCloud2, '/new_points_topic', 10)

        # Initialize bounding boxes
        self.bounding_boxes = []

        # Subscriber to the ZED bounding box topic
        self.box_subscription = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self.listener_callback_bounding_boxes,
            10
        )
        self.box_subscription  # prevent unused variable warning

    def listener_callback_pointcloud(self, msg):
        # Log the reception of a point cloud message
        self.get_logger().info('Received point cloud data')

        # Get point cloud data as numpy array
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        filtered_points = []

        # Filter points based on bounding boxes
        for point in points:
            x, y, z = point

            # Check if the original point is within the bounding boxes
            if self.is_point_in_bounding_boxes(-x, -y, z):
                # Rotate the point 180 degrees around the Z-axis
                transformed_x = x
                transformed_y = y
                transformed_z = z
                filtered_points.append((transformed_x, transformed_y, transformed_z))

        # Create a new point cloud message with filtered and transformed points
        if filtered_points:
            self.publish_filtered_points(filtered_points, msg.header)

    def publish_filtered_points(self, points, original_header):
        # Create a new header
        new_header = Header()
        new_header.stamp = self.get_clock().now().to_msg()
        new_header.frame_id = original_header.frame_id  # Use the original frame_id

        # Create the PointCloud2 message
        filtered_pc = pc2.create_cloud_xyz32(new_header, points)

        # Publish the filtered point cloud data
        self.publisher.publish(filtered_pc)
        self.get_logger().info(f'Published transformed point cloud data with {len(points)} points')

    def is_point_in_bounding_boxes(self, x, y, z):
        # Check if the point (x, y, z) is within any of the bounding boxes
        for box in self.bounding_boxes:
            min_x, min_y, min_z, max_x, max_y, max_z = box
            if (min_x <= x <= max_x and
                min_y <= y <= max_y and
                min_z <= z <= max_z):
                return True
        return False

    def listener_callback_bounding_boxes(self, msg):
        # Log the reception of bounding boxes
        self.get_logger().info(f'Received bounding box data with count: {len(msg.objects)}')

        # Update the bounding boxes list
        self.bounding_boxes.clear()
        for obj in msg.objects:
            if hasattr(obj, 'bounding_box_3d'):
                # Extract the corners of the 3D bounding box
                corners = obj.bounding_box_3d.corners

                # Calculate min and max extents from the corners
                xs = [corner.kp[0] for corner in corners]  # X coordinates
                ys = [corner.kp[1] for corner in corners]  # Y coordinates
                zs = [corner.kp[2] for corner in corners]  # Z coordinates

                # Define the bounding box limits
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)
                min_z, max_z = min(zs), max(zs)

                # Store the bounding box as a tuple
                self.bounding_boxes.append((min_x, min_y, min_z, max_x, max_y, max_z))

                self.get_logger().info(
                    f'Added bounding box: min ({min_x}, {min_y}, {min_z}), max ({max_x}, {max_y}, {max_z})'
                )

def main(args=None):
    rclpy.init(args=args)
    ouster_bridge_node = OusterBridgeNode()
    rclpy.spin(ouster_bridge_node)

    ouster_bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
