#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <unistd.h> // For usleep

class BeepNode
{
public:
    BeepNode()
    {
        // Initialize subscribers
        pitch_sub_ = nh_.subscribe("/beep/pitch", 10, &BeepNode::pitchCallback, this);
        volume_sub_ = nh_.subscribe("/beep/volume", 10, &BeepNode::volumeCallback, this);
        duty_cycle_sub_ = nh_.subscribe("/beep/duty_cycle", 10, &BeepNode::dutyCycleCallback, this);
        channel_sub_ = nh_.subscribe("/beep/channel", 10, &BeepNode::channelCallback, this);

        // Initialize default values
        pitch_ = 440.0;  // A4 note
        volume_ = 50;    // 50% volume
        duty_cycle_ = 50; // 50% duty cycle
        channel_ = 0;    // Default audio channel
        num_beeps_ = 5;  // Number of beeps
        interval_ = 200000; // Interval between beeps in microseconds (200ms)
    }

    void spin()
    {
        ros::Rate rate(10);  // 10 Hz
        while (ros::ok())
        {
            // Play beep sound
            playBeep(pitch_, volume_, duty_cycle_, channel_, num_beeps_, interval_);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pitch_sub_;
    ros::Subscriber volume_sub_;
    ros::Subscriber duty_cycle_sub_;
    ros::Subscriber channel_sub_;

    float pitch_;
    int volume_;
    int duty_cycle_;
    int channel_;
    int num_beeps_;
    int interval_;

    void pitchCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        pitch_ = msg->data;
        ROS_INFO("Received pitch: %f", pitch_);
    }

    void volumeCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        volume_ = msg->data;
        ROS_INFO("Received volume: %d", volume_);
    }

    void dutyCycleCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        duty_cycle_ = msg->data;
        ROS_INFO("Received duty cycle: %d", duty_cycle_);
    }

    void channelCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        channel_ = msg->data;
        ROS_INFO("Received channel: %d", channel_);
    }

    void playBeep(float pitch, int volume, int duty_cycle, int channel, int num_beeps, int interval)
    {
        for (int i = 0; i < num_beeps; ++i) {
            // Construct the sox command to play the beep
            std::ostringstream command;
            command << "play -n synth 0.1 sine " << pitch
                    << " vol " << (volume / 100.0);

            // Add channel control
            if (channel == 0) {
                command << " remix 1";  // Left channel
            } else if (channel == 1) {
                command << " remix 2";  // Right channel
            }

            // Execute the sox command
            system(command.str().c_str());

            ROS_INFO("Playing beep sound with pitch: %f Hz, volume: %d%%, duty cycle: %d%%, channel: %d",
                     pitch, volume, duty_cycle, channel);

            // Pause between beeps
            usleep(interval);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "play_beep_node");

    BeepNode beepNode;
    beepNode.spin();

    return 0;
}
