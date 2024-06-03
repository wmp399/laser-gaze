#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from scipy.linalg import svd

class PointCloudFlatnessCalculator(Node):
    def __init__(self):
        super().__init__('point_cloud_flatness_calculator')

        # Subscribe to the PointCloud2 topic
        self.point_cloud_sub = self.create_subscription(PointCloud2, '/scan', self.point_cloud_callback, 10)

    def point_cloud_callback(self, msg):
        # Convert ROS PointCloud2 message to numpy array
        points = self.point_cloud_to_numpy(msg)

        # Compute flatness
        flatness = self.calculate_flatness(points)

        self.get_logger().info("Flatness of the point cloud: {}".format(flatness))

    def point_cloud_to_numpy(self, msg):
        # Convert ROS PointCloud2 message to numpy array
        # You will need to implement this function based on your specific message format
        # Here's a dummy implementation assuming xyz data with 3 channels
        points = np.array([msg.data[i:i+3] for i in range(0, len(msg.data), 12)])

        return points

    def calculate_flatness(self, points):
        # Compute centroid of the points
        centroid = np.mean(points, axis=0)

        # Subtract centroid from points
        centered_points = points - centroid

        # Perform Singular Value Decomposition (SVD)
        _, _, V = svd(centered_points)

        # Extract the normal vector of the plane from the last row of V
        normal = V[-1, :]

        # Calculate distances from points to the plane
        distances = np.abs(np.dot(centered_points, normal))

        # Calculate flatness as the standard deviation of distances
        flatness = np.std(distances)

        return flatness

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFlatnessCalculator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()