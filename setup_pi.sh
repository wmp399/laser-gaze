# check for updates and apply them
echo "check for updates and apply them"
sudo apt update
sudo apt upgrade -y

# get laser-gaze files
echo "get laser-gaze files"
git clone https://github.com/wmp399/laser-gaze.git

# get rplidar driver
echo "get rplidar driver"
cd laser-gaze
git clone https://github.com/babakhani/rplidar_ros2
# get unitree lidar driver
echo "get unitree lidar driver"
git clone https://github.com/unitreerobotics/unilidar_sdk/tree/main
cd ..

# Get docker and install
echo "Get docker and install"
sudo apt-get install docker

"Build ROS-Base"
cd laser-gaze/docker-iron
docker build -t ros2 .

docker-compose up -d

echo "done."
echo "type `docker run -it docker-iron_ros2_1` to run ros2"