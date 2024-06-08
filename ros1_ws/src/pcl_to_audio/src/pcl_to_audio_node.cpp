#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

class PCLtoAudioNode
{
public:
    PCLtoAudioNode()
    {
        // Initialize the subscriber
        sub_ = nh.subscribe("input_topic", 10, &PCLtoAudioNode::callback, this);

        // Initialize the publishers
        pitch_pub_ = nh.advertise<std_msgs::Float32>("pitch_output", 10);
        volume_pub_ = nh.advertise<std_msgs::Float32>("volume_output", 10);
        channel_pub_ = nh.advertise<std_msgs::Int32>("channel_output", 10);
        interval_pub_ = nh.advertise<std_msgs::Float32>("interval_output", 10);
    }

    // Function to handle node spin
    void spin()
    {
        // Spin the node
        ros::spin();
    }

private:
    ros::NodeHandle nh;

    ros::Publisher pitch_pub_;
    ros::Publisher volume_pub_;
    ros::Publisher channel_pub_;
    ros::Publisher interval_pub_;

    ros::Subscriber sub_;

    // Callback function to process data received from the subscribed topic
    void callback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);

        float closest_point[3] = {0, 0, 0};
        float min_distance = 1000;

        // Iterate over all points in the point cloud to find the closest one
        for (const auto& point : cloud->points)
        {
            // Calculate the distance from the origin
            float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (distance < min_distance) {
                min_distance = distance;
                closest_point[0] = point.x;
                closest_point[1] = point.y;
                closest_point[2] = point.z;
            }
        }

        float min_pitch = 1000;
        float max_pitch = 200;

        float angle = abs(cos(closest_point[0]/closest_point[2]))*180/3.14159;
        bool angle_sign = closest_point[0]>0;

        float pitch = 200 + (1000 - 200) * (angle) / (45.0);

        // Create and publish the pitch
        std_msgs::Float32 pitch_msg;
        pitch_msg.data = pitch;
        pitch_pub_.publish(pitch_msg);

        // Create and publish the volume
        std_msgs::Float32 volume_msg;
        volume_msg.data = 1.0;
        volume_pub_.publish(volume_msg);

        // Create and publish the channel
        std_msgs::Int32 channel_msg;
        if (angle < 5) channel_msg.data = 0;
        else if (angle_sign) channel_msg.data = 1;
        else channel_msg.data = -1;
        channel_pub_.publish(channel_msg);

        float interval = 0.1 + (2.0 - 0.1) * (min_distance / 10.0);

        // Create and publish the interval
        std_msgs::Float32 interval_msg;
        interval_msg.data = interval;
        interval_pub_.publish(interval_msg);
    }
};

int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "pcl_to_audio_node");

    // Create an instance of the SimpleNode class
    PCLtoAudioNode pcl_to_audio_node;

    // Run the node
    pcl_to_audio_node.spin();

    return 0;
}
