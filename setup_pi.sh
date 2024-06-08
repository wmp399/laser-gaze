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
sudo apt install -y ros-noetic-desktop
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential 
sudo rosdep update
sudo rosdep init

# Install dependencies
echo "install dependencies"
sudo apt install -y ros-noetic-pcl*
sudo apt install -y libsndfile1-dev
sudo apt install -y libao-dev

# get laser-gaze files
echo "get laser-gaze files and submodules"
# git clone https://github.com/wmp399/laser-gaze.git
# if you have this script, you have probably already cloned the laser-gaze repository.
cd laser-gaze
git submodule update --init --recursive

# build unitree lidar sdk
echo "build unitree lidar sdk"
cd ~/laser-gaze/unilidar_sdk/unitree_lidar_ros
catkin_make

# build LaserGaze
echo "build LaserGaze"
cd ~/laser-gaze/ros1_ws
catkin_make

source ~/laser-gaze/ros1_ws/devel/setup.bash
source ~/laser-gaze/unilidar_sdk/unitree_lidar_ros/devel/setup.bash --extend