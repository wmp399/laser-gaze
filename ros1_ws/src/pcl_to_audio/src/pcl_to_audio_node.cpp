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

        // Get the launch parameters
        nh.param<double>("/min_pitch", min_pitch_, 20.0);
        nh.param<double>("/max_pitch", max_pitch_, 20000.0);
        nh.param<double>("/min_angle", min_angle_, -180.0);
        nh.param<double>("/max_angle", max_angle_, 180.0);

        nh.param<double>("/center_band", center_band_, 5.0);

        nh.param<double>("/min_interval", min_interval_, 0.05);
        nh.param<double>("/max_interval", max_interval_, 5.00);
        nh.param<double>("/min_distance", min_distance_, 0.00);
        nh.param<double>("/max_distance", max_distance_, 20.00);
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

    double min_pitch_;
    double max_pitch_;
    double min_angle_;
    double max_angle_;

    double center_band_;

    double min_interval_;
    double max_interval_;
    double min_distance_;
    double max_distance_;

    // Callback function to process data received from the subscribed topic
    void callback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud);

        float closest_point[3] = {0, 0, 0};
        float closest_point_dist = max_distance_;

        // Iterate over all points in the point cloud to find the closest one
        for (const auto& point : cloud->points)
        {
            // Calculate the distance from the origin
            float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (distance < closest_point_dist) {
                closest_point_dist = distance;
                closest_point[0] = point.x;
                closest_point[1] = point.y;
                closest_point[2] = point.z;
            }
        }

        float angle = abs(atan2(closest_point[0],closest_point[2]))*180/3.14159;
        if (angle > max_angle_) angle = max_angle_;
        bool angle_sign = closest_point[0]>0;

        float pitch = min_pitch_ + (max_pitch_ - min_pitch_) * (max_angle_ - angle) / (max_angle_);

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
        if (angle < center_band_) channel_msg.data = 0;
        else if (angle_sign) channel_msg.data = 1;
        else channel_msg.data = -1;
        channel_pub_.publish(channel_msg);

        float interval = min_interval_ + (max_interval_ - min_interval_) * (closest_point_dist / (max_distance_ - min_distance_));

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
