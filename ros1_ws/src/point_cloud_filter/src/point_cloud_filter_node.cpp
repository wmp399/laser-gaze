/* This file was originally generated by ChatGPT (then modified), 
meaning that it was generated by referencing various unknown sources. 
I would love to cite my work here, but ChatGPT does not provide 
information on what the responses are generated from :( */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
class PointCloudFilter {
public:
    PointCloudFilter() {
        // Initialize ROS node
        ros::NodeHandle nh;

        // Subscribe to the input PointCloud2 topic
        pointcloud_sub = nh.subscribe("/pcl_filter_input", 1, &PointCloudFilter::pointCloudCallback, this);

        // Advertise the output PointCloud2 topic
        pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_filter_output", 1);

        // Get the ROS parameters
        nh.param<double>( "/x_min", x_min_, 0.0);
        nh.param<double>("/x_max", x_max_, 1.0);
        nh.param<double>("/y_min", y_min_, 0.0);
        nh.param<double>("/y_max", y_max_, 1.0);
        nh.param<double>("/z_min", z_min_, 0.0);
        nh.param<double>("/z_max", z_max_, 1.0);
    }

private:
    ros::Subscriber pointcloud_sub;
    ros::Publisher pointcloud_pub;
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
    double z_min_;
    double z_max_;

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
             
        // Convert PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        ROS_INFO("X MIN: %f", x_min_);

        if (cloud->points.empty())
        {
            ROS_INFO("No points in the point cloud.");
            return;
        }

        // Perform PassThrough filtering to extract the region of interest
        pcl::PassThrough<pcl::PointXYZ> pass;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // Filter along x-axis
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(x_min_, x_max_);
        pass.filter(*cloud_filtered);

        // Filter along y-axis
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_min_, y_max_);
        pass.filter(*cloud_filtered);

        // Filter along z-axis
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*cloud_filtered);

        // Convert PCL PointCloud back to PointCloud2 message
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);

        // Publish the filtered PointCloud2 message
        output.header = cloud_msg->header;
        pointcloud_pub.publish(output);
    }
};

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "pointcloud_filter");

    // Create the PointCloudFilter object
    PointCloudFilter pointCloudFilter;

    // Spin to process callbacks
    ros::spin();

    return 0;
}
