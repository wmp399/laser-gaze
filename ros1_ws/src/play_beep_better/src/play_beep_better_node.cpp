#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <portaudio.h>
#include <vector>
#include <thread>

const int SAMPLE_RATE_ = 44100;
const int FRAMES_PER_BUFFER_ = 64;

PaStream *stream_;
std::vector<float> beep_data_(SAMPLE_RATE_*2);

class BeepBetterNode {
    public:
        BeepBetterNode() {
            // Initialize subscribers
            pitch_sub_ = nh_.subscribe("/pitch", 10, &BeepBetterNode::pitchCallback, this);
            volume_sub_ = nh_.subscribe("/volume", 10, &BeepBetterNode::volumeCallback, this);
            channel_sub_ = nh_.subscribe("/channel", 10, &BeepBetterNode::channelCallback, this);
            interval_sub_ = nh_.subscribe("/interval", 10, &BeepBetterNode::intervalCallback, this);

            // Initialize default values
            pitch_ = 440.0;  // A4 note
            volume_ = 0.5;   // 50% volume
            channel_ = 0;    // Default audio channel -- L=-1, C=0, R=+1
            interval_ = 1.0; // Interval between beeps in seconds (1.0s)
            length_ = interval_ / 4.0;

            timer_duration_ = 0.005; // 10 milliseconds
            cycles_since_last_beep_ = 0;

            Pa_Initialize();
            restartStream();

            // Initialize the timer with a default interval of 10 milli-seconds
            timer_ = nh_.createTimer(ros::Duration(timer_duration_), &BeepBetterNode::timerCallback, this);
        }

        ~BeepBetterNode() {
            Pa_StopStream(stream_);
            Pa_CloseStream(stream_);
            Pa_Terminate();
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
        float length_;

        float timer_duration_;
        int cycles_since_last_beep_;

        void pitchCallback(const std_msgs::Float32::ConstPtr& msg) {
            pitch_ = msg->data;
            if (pitch_ < 20) pitch_ = 20;
            if (pitch_ > 20000) pitch_ = 20000;
            ROS_INFO("Received pitch: %f", pitch_);
        }

        void volumeCallback(const std_msgs::Float32::ConstPtr& msg) {
            volume_ = msg->data;
            if (volume_ < 0.0) volume_ = 0.0;
            if (volume_ > 1.0) volume_ = 1.0;
            ROS_INFO("Received volume: %f", volume_);
        }

        void channelCallback(const std_msgs::Int32::ConstPtr& msg) {
            channel_ = msg->data;
            if (channel_ < -1) channel_ = -1;
            if (channel_ > 1) channel_ = 1; 
            ROS_INFO("Received channel: %d", channel_);
        }

        void intervalCallback(const std_msgs::Float32::ConstPtr& msg) {
            interval_ = msg->data;
            if (interval_ < 0.02) interval_ = 0.02;
            if (interval_ > 10.0) interval_ = 10.0;
            length_ = interval_ / 4.0;
            if (length_ > 0.5) length_ = 0.5;
            ROS_INFO("Received interval: %f", interval_);
        }

        void generateBeep(float frequency) {
            beep_data_.resize(SAMPLE_RATE_*2);
            for (int i = 0; i < SAMPLE_RATE_; ++i) {
                float sample = volume_ * std::sin((2.f * 3.14159 * i * frequency) / SAMPLE_RATE_);
                if (channel_ <= 0) {
                    beep_data_[2*i] = sample;
                } else beep_data_[2*i] = 0;
                if (channel_ >= 0) {
                    beep_data_[2*i+1] = sample;
                } else beep_data_[2*i+1] = 0;
            }
        }

        static int audioCallback(const void *inputBuffer, void *outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags, void *userData) {
            float *out = (float*)outputBuffer;
            static size_t offset = 0;
            
            for (unsigned int i = 0; i < framesPerBuffer * 2; ++i) {
                out[i] = beep_data_[offset];
                offset = (offset+1) % (SAMPLE_RATE_*2);
            }

            return paContinue;
        }

        void restartStream() {
            Pa_StopStream(stream_);
            Pa_CloseStream(stream_);
            Pa_OpenDefaultStream(&stream_, 0, 2, paFloat32, SAMPLE_RATE_, FRAMES_PER_BUFFER_, audioCallback, nullptr);
            Pa_StartStream(stream_);
            generateBeep(pitch_);
        }

        void timerCallback(const ros::TimerEvent&) {
            if (cycles_since_last_beep_ >= interval_ / timer_duration_) {
                cycles_since_last_beep_ = 0;
                generateBeep(pitch_);
                Pa_StopStream(stream_);
                Pa_StartStream(stream_);
            } else if (cycles_since_last_beep_ >= length_ / timer_duration_) {
                //std::fill(beep_data_.begin(), beep_data_.end(), 0);
                Pa_StopStream(stream_);
                cycles_since_last_beep_++;
            } else {
                cycles_since_last_beep_++;
            }
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "play_beep_better_node");
    BeepBetterNode beep_better_node;
    ros::spin();

    return 0;
}
