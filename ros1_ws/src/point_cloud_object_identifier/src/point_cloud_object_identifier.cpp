/* This file was originally generated by ChatGPT (then modified), 
meaning that it was generated by referencing various unknown sources. 
I would love to cite my work here, but ChatGPT does not provide 
information on what the responses are generated from :( */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>

class PointCloudObjectIdentifier
{
public:
    PointCloudObjectIdentifier()
    {
        // Subscribe to the input point cloud topic
        sub_ = nh_.subscribe("/pcl_cluster_input", 1, &PointCloudObjectIdentifier::pointCloudCallback, this);
        // Advertise the output point cloud topic for clusters
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_cluster_output", 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);

        // Perform Euclidean Cluster Extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02); // 2cm
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // Create a new PointCloud for the clusters
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusters(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& indices : cluster_indices)
        {
            for (const auto& index : indices.indices)
            {
                clusters->points.push_back(cloud->points[index]);
            }
        }

        // Convert the PCL PointCloud to ROS PointCloud2 and publish
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*clusters, output);
        output.header = input->header;
        pub_.publish(output);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_object_identifier");
    PointCloudObjectIdentifier node;
    ros::spin();
    return 0;
}
