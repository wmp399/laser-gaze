#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/impl/common.hpp>
#include <Eigen/Dense>

class PointCloudFlatnessCalculator
{
public:
    PointCloudFlatnessCalculator()
    {
        // Initialize node
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // Parameters for the bounding box
        pnh.param("min_x", min_x_, -1.0);
        pnh.param("max_x", max_x_, 1.0);
        pnh.param("min_y", min_y_, -1.0);
        pnh.param("max_y", max_y_, 1.0);
        pnh.param("min_z", min_z_, -1.0);
        pnh.param("max_z", max_z_, 1.0);

        // Subscribe to the PointCloud2 topic
        point_cloud_sub_ = nh.subscribe("/point_cloud", 10, &PointCloudFlatnessCalculator::pointCloudCallback, this);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // Convert ROS PointCloud2 message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Extract points within the bounding box
        pcl::PointCloud<pcl::PointXYZ>::Ptr region_points(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : cloud->points)
        {
            if (point.x >= min_x_ && point.x <= max_x_ &&
                point.y >= min_y_ && point.y <= max_y_ &&
                point.z >= min_z_ && point.z <= max_z_)
            {
                region_points->points.push_back(point);
            }
        }

        if (region_points->points.empty())
        {
            ROS_INFO("No points in the specified region.");
            return;
        }

        // Compute flatness
        double flatness = calculateFlatness(region_points);
        ROS_INFO("Flatness of the point cloud in the specified region: %f", flatness);
    }

    double calculateFlatness(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points)
    {
        // Compute centroid of the points
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*points, centroid);

        // Subtract centroid from points
        Eigen::MatrixXf centered_points(points->points.size(), 3);
        for (size_t i = 0; i < points->points.size(); ++i)
        {
            centered_points(i, 0) = points->points[i].x - centroid[0];
            centered_points(i, 1) = points->points[i].y - centroid[1];
            centered_points(i, 2) = points->points[i].z - centroid[2];
        }

        // Perform Singular Value Decomposition (SVD)
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(centered_points, Eigen::ComputeThinU | Eigen::ComputeThinV);

        // Extract the normal vector of the plane from the last column of V
        Eigen::Vector3f normal = svd.matrixV().col(2);

        // Calculate distances from points to the plane
        Eigen::VectorXf distances(points->points.size());
        for (size_t i = 0; i < points->points.size(); ++i)
        {
            distances(i) = std::abs(centered_points.row(i).dot(normal));
        }

        // Calculate flatness as the standard deviation of distances
        return std::sqrt(distances.array().square().sum() / distances.size());
    }

private:
    ros::Subscriber point_cloud_sub_;
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_flatness_calculator");
    PointCloudFlatnessCalculator node;
    ros::spin();
    return 0;
}
