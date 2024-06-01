#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from sklearn.cluster import DBSCAN

class PointCloudSegmenter(Node):
    def __init__(self):
        super().__init__("point_cloud_segmenter")

        # Subscribe to the input point cloud topic
        self.subscription = self.create_subscription(PointCloud2, "/input_point_cloud", self.point_cloud_callback, 10)

        # Advertise a new topic for publishing segmented clusters
        self.publisher = self.create_publisher(PointCloud2, "/segmented_clusters", 10)

    def point_cloud_callback(self, msg):
        # Convert ROS PointCloud2 message to numpy array
        points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])

        # DBSCAN clustering
        eps = 0.1  # neighborhood distance
        min_samples = 10  # minimum number of points in neighborhood
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)

        # Get labels assigned to each point by DBSCAN
        labels = db.labels_

        # Get unique labels (excluding noise points, labeled as -1)
        unique_labels = set(labels) - {-1}

        # Publish each cluster as a separate PointCloud2 message
        for label in unique_labels:
            mask = labels == label
            cluster_points = points[mask]
            cluster_msg = pc2.create_cloud_xyz32(msg.header, cluster_points)
            self.publisher.publish(cluster_msg)

def main(args=None):
    rclpy.init(args=args)
    point_cloud_segmenter = PointCloudSegmenter()
    rclpy.spin(point_cloud_segmenter)
    point_cloud_segmenter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()