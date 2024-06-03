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
        pointcloud_sub = nh.subscribe("/input_pointcloud", 1, &PointCloudFilter::pointCloudCallback, this);

        // Advertise the output PointCloud2 topic
        pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_pointcloud", 1);
    }

private:
    ros::Subscriber pointcloud_sub;
    ros::Publisher pointcloud_pub;

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        // Convert PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // Perform PassThrough filtering to extract the region of interest
        pcl::PassThrough<pcl::PointXYZ> pass;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // Filter along x-axis
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.0, 1.0); // Change these limits according to your region of interest
        pass.filter(*cloud_filtered);

        // Filter along y-axis
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-0.5, 0.5); // Change these limits according to your region of interest
        pass.filter(*cloud_filtered);

        // Filter along z-axis
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.5); // Change these limits according to your region of interest
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
