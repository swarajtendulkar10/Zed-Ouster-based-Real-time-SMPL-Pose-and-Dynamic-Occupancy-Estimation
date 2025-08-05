#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class LidarToDensityGrid(Node):
    def __init__(self):
        super().__init__('lidar_to_density_grid')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/new_points_topic',  # Update with your LiDAR point cloud topic
            self.pointcloud_callback,
            10)
        self.occupancy_pub = self.create_publisher(OccupancyGrid, '/density_grid', 10)

        # Parameters for the occupancy grid
        self.grid_resolution = 0.1  # Resolution in meters per cell
        self.grid_width = 1000  # Number of cells in the x-direction
        self.grid_height = 1000  # Number of cells in the y-direction
        self.origin_x = -5.0  # Map's origin x position (meters)
        self.origin_y = -5.0  # Map's origin y position (meters)
        self.max_density_value = 10  # Maximum point density for normalization

    def pointcloud_callback(self, msg):
        # Initialize an empty occupancy grid with zero density
        density_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)

        # Iterate through the point cloud
        for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = point
            if z > -0.1 and z < 0.5:  # Adjust the height filter for ground points
                # Apply a 180-degree rotation around the Z-axis
                rotated_x = -x
                rotated_y = -y

                # Convert rotated point to grid cell
                grid_x = int((rotated_x - self.origin_x) / self.grid_resolution)
                grid_y = int((rotated_y - self.origin_y) / self.grid_resolution)

                if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                    density_grid[grid_y, grid_x] += 1  # Increment the density of the cell

        # Normalize density to values between 0 and 100
        density_grid = np.clip(density_grid, 0, self.max_density_value)
        normalized_grid = (density_grid / self.max_density_value) * 100
        normalized_grid = normalized_grid.astype(np.int8)

        # Create the OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = 'os_sensor'  # Adjust frame_id as needed
        occupancy_grid.info.resolution = self.grid_resolution
        occupancy_grid.info.width = self.grid_width
        occupancy_grid.info.height = self.grid_height
        occupancy_grid.info.origin.position.x = self.origin_x
        occupancy_grid.info.origin.position.y = self.origin_y
        occupancy_grid.data = normalized_grid.flatten().tolist()

        # Publish the occupancy grid
        self.occupancy_pub.publish(occupancy_grid)
        self.get_logger().info('Published rotated density grid')

def main(args=None):
    rclpy.init(args=args)
    node = LidarToDensityGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
