# LaserGaze
The LaserGaze system takes in data from a 4D LIDAR Scanner and provides auditory feedback to assist persons with visual impairments drive a vehicle. 

## Deployment
This program is intended to run on a Raspberry Pi 4 using ROS Noetic (not ROS2). The supported and test operating system is Ubuntu 20.04 LTS Server 64-bt.

## Dependencies
Running LaserGaze requires a few depedencies - ROS Noetic, the Unitree Robotics SDK, and the Unitree Robotics Point-LIO Example. Running the script setup_pi.sh will install these dependencies automatically. 
In addition, for ease of development or to run the visualization, a light weight desktop should be installed. One good option is lubuntu. It can be installed using:
sudo apt update
sudo apt upgrade -y
sudo apt install lubuntu-desktop

## Starting LaserGaze
source ~/laser-gaze/ros1_ws/devel/setup.bash
source ~/unilidar_sdk/unitree_lidar_ros/devel/setup.bash --extend
roslaunch laser_gaze laser_gaze.launch

TODO: Should roscore be started independently? Is this required to access the LaserGaze topics remotely?

## Running LaserGaze Visualization
### On the same Rasperry Pi as the Core LaserGaze
NOTE: This will only work when there is a GUI running. The visualization cannot be run from an SSH connection.
source ~/laser-gaze/ros1_ws/devel/setup.bash
source ~/unilidar_sdk/unitree_lidar_ros/devel/setup.bash --extend
roslaunch laser_gaze visualization.launch

### On a Remote Laptop or Raspberry Pi
source ~/laser-gaze/ros1_ws/devel/setup.bash
source ~/unilidar_sdk/unitree_lidar_ros/devel/setup.bash --extend
export ROS_HOSTNAME={ip address or LaserGaze Pi}
export ROS_MASTER_URI=http://{ip address or LaserGaze Pi}:11311
roslaunch laser_gaze visualization.launch


## Useful Links / References: 
ROS Documentation: https://wiki.ros.org/ 

ROS Installation Instructions for Ubuntu: https://wiki.ros.org/noetic/Installation/Ubuntu

Unitree Lidar L1 SDK: https://github.com/unitreerobotics/unilidar_sdk/tree/main

Unitree LIO Implementation: https://github.com/unitreerobotics/point_lio_unilidar/tree/main

Learn ROS Course by Kevin McAleer (including getting ROS running in Docker): https://www.kevsrobots.com/learn/learn_ros/

Running a ROS Launcher Automatically on Boot: https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/

Adding additional WIFI networks manually: https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line
NOTE: Use "ip a" to get the device name used in the config file.