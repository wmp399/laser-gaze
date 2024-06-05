/* This file was originally generated by ChatGPT (then modified), 
meaning that it was generated by referencing various unknown sources. 
I would love to cite my work here, but ChatGPT does not provide 
information on what the responses are generated from :( */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
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

        // Subscribe to the PointCloud2 topic
        point_cloud_sub_ = nh.subscribe("/pcl_flatness_input", 10, &PointCloudFlatnessCalculator::pointCloudCallback, this);
        flatness_pub_ = nh.advertise<std_msgs::Float64>("/pcl_flatness_output", 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // Convert ROS PointCloud2 message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->points.empty())
        {
            ROS_INFO("No points in the point cloud.");
            return;
        }

        // Compute flatness
        std_msgs::Float64 output;
        output.data = calculateFlatness(cloud);
        flatness_pub_.publish(output);
        //ROS_INFO("Flatness of the point cloud: %f", output.data);
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
    ros::Publisher flatness_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_flatness_calculator");
    PointCloudFlatnessCalculator node;
    ros::spin();
    return 0;
}