#include "ros/ros.h"
#include "std_msgs/String.h"

class PCLtoAudioNode
{
public:
    PCLtoAudioNode()
    {
        // Initialize the ROS node handle
        ros::NodeHandle nh;

        // Initialize the publisher
        pub_ = nh.advertise<std_msgs::String>("output_topic", 10);

        // Initialize the subscriber
        sub_ = nh.subscribe("input_topic", 10, &PCLtoAudioNode::callback, this);
    }

    // Callback function to process data received from the subscribed topic
    void callback(const std_msgs::String::ConstPtr& msg)
    {
        // Log the received message
        ROS_INFO("Received: [%s]", msg->data.c_str());

        // Create and publish a response message
        std_msgs::String output_msg;
        output_msg.data = "Processed: " + msg->data;
        pub_.publish(output_msg);
    }

    // Function to handle node spin
    void spin()
    {
        // Spin the node
        ros::spin();
    }

private:
    ros::Publisher pub_;
    ros::Subscriber sub_;
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
