#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <boost/thread/mutex.hpp>

class BeepNode
{
public:
    BeepNode()
    {
        // Initialize subscribers
        pitch_sub_ = nh_.subscribe("/pitch", 10, &BeepNode::pitchCallback, this);
        volume_sub_ = nh_.subscribe("/volume", 10, &BeepNode::volumeCallback, this);
        channel_sub_ = nh_.subscribe("/channel", 10, &BeepNode::channelCallback, this);
        interval_sub_ = nh_.subscribe("/interval", 10, &BeepNode::intervalCallback, this);

        // Initialize the timer with a default interval of 1 second
        timer_ = nh_.createTimer(ros::Duration(1.0), &BeepNode::timerCallback, this);

        // Initialize default values
        pitch_ = 440.0;  // A4 note
        volume_ = 0.5;    // 50% volume
        channel_ = 0;    // Default audio channel -- L=-1, C=0, R=+1
        interval_ = 1.0; // Interval between beeps in seconds (1.0s)
    }

private:
    ros::NodeHandle nh_;
    
    ros::Subscriber pitch_sub_;
    ros::Subscriber volume_sub_;
    ros::Subscriber channel_sub_;
    ros::Subscriber interval_sub_;

    ros::Timer timer_;
    
    float pitch_;
    float volume_;
    int channel_;
    float interval_;

    boost::mutex mutex_;

    void pitchCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        pitch_ = msg->data;
        if (pitch_ < 20) pitch_ = 20;
        if (pitch_ > 20000) pitch_ = 20000;
        ROS_INFO("Received pitch: %f", pitch_);
    }

    void volumeCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        volume_ = msg->data;
        if (volume_ < 0.0) volume_ = 0.0;
        if (volume_ > 1.0) volume_ = 1.0;
        ROS_INFO("Received volume: %f", volume_);
    }

    void channelCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        channel_ = msg->data;
        if (channel_ < -1) channel_ = -1;
        if (channel_ > 1) channel_ = 1; 
        ROS_INFO("Received channel: %d", channel_);
    }

    void intervalCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(mutex_);
        interval_ = msg->data;
        if (interval_ < 0.01) interval_ = 0.01;
        if (interval_ > 10.0) interval_ = 10.0;
        ROS_INFO("Received interval: %f", interval_);
        timer_.stop();
        timer_ = nh_.createTimer(ros::Duration(interval_), &BeepNode::timerCallback, this);
        ROS_INFO("Interval updated to: %f seconds", interval_);      
    }

    void timerCallback(const ros::TimerEvent&)
    {
        // Construct the sox command to play the beep
        std::ostringstream command;
        command << "play -V -r 48000 -n -b 16 -c 2 synth 0.1 sin " << pitch_ << " "; 
        command << "vol " << volume_;

        // Add channel control
        if (channel_ == -1) {
            command << " remix 1 0";    // Left channel
        } else if (channel_ == 0) {
            command << " remix 1 1";    // Both channels
        } else if (channel_== 1) {
            command << " remix 0 1";    // Right channel
        }

        // Execute the sox command
        system(command.str().c_str());
        ROS_INFO("Playing beep sound with pitch: %f Hz, volume: %f, channel: %d, interval: %f",
                     pitch_, volume_, channel_, interval_);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "play_beep_node");
    BeepNode beep_node;
    ros::spin();
    return 0;
}
