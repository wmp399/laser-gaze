# check for updates and apply them
echo "check for updates and apply them"
sudo apt update
sudo apt upgrade -y

# Open firewall ports for ROS and SSH
echo "open SSH and ROS ports in firewall"
sudo ufw allow 22
sudo ufw allow 11311

# Install ROS Noetic
echo "installing ROS Noetic"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep update
sudo rosdep init

# Install dependencies
echo "install dependencies"
sudo apt install ros-noetic-pcl*
sudo apt install libsndfile1-dev
sudo apt install libao-dev

# get laser-gaze files
echo "get laser-gaze files and submodules"
git clone https://github.com/wmp399/laser-gaze.git
cd laser-gaze
git submodule update --init --recursive

# get unitree lidar driver
echo "get unitree lidar driver"
cd
git clone https://github.com/unitreerobotics/unilidar_sdk/tree/main

# build unitree lidar driver
echo "build unitree lidar driver"
cd ~/unilidar_sdk/unitree_lidar_ros
source devel/setup.bash
catkin_make

# build LaserGaze
echo "build LaserGaze"
cd ~/laser-gaze/ros1_ws
source devel/setup.bash
source ~/unilidar_sdk/unitree_lidar_ros/devel/setup.bash --extend
catkin_make
