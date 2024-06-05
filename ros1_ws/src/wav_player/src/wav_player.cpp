#include <ros/ros.h>
#include <ao/ao.h>
#include <sndfile.h>
#include <std_msgs/Bool.h>

class WavPlayer {
public:
    WavPlayer(ros::NodeHandle& nh) {
        sub_ = nh.subscribe("play_wav_trigger", 10, &WavPlayer::triggerCallback, this);
        nh.param<std::string>("wav_file", wav_file_, "test.wav");
    }

    void triggerCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) {
            ROS_INFO("Playing wav file: %s", wav_file_.c_str());
            playWavFile(wav_file_);
        }
    }

    void playWavFile(const std::string &filename) {
        SF_INFO sfinfo;
        SNDFILE *sndfile = sf_open(filename.c_str(), SFM_READ, &sfinfo);

        if (!sndfile) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return;
        }

        ao_initialize();
        int defaultDriver = ao_default_driver_id();
        ao_sample_format format;

        format.bits = 16;
        format.channels = sfinfo.channels;
        format.rate = sfinfo.samplerate;
        format.byte_format = AO_FMT_LITTLE;
        format.matrix = 0;

        ao_device *device = ao_open_live(defaultDriver, &format, NULL);
        if (!device) {
            ROS_ERROR("Failed to open audio device.");
            sf_close(sndfile);
            ao_shutdown();
            return;
        }

        const int bufferSize = 4096;
        short buffer[bufferSize];

        while (true) {
            sf_count_t count = sf_read_short(sndfile, buffer, bufferSize);
            if (count == 0) break;

            ao_play(device, reinterpret_cast<char*>(buffer), static_cast<uint_32>(count * sizeof(short)));
        }

        ao_close(device);
        sf_close(sndfile);
        ao_shutdown();
    }

private:
    ros::Subscriber sub_;
    std::string wav_file_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "wav_player");
    ros::NodeHandle nh;

    WavPlayer wav_player(nh);

    ros::spin();

    return 0;
}
