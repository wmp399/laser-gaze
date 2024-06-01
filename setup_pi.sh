# check for updates and apply them
echo "check for updates and apply them"
sudo apt update
sudo apt upgrade -y

# get laser-gaze files
echo "get laser-gaze files"
git clone https://github.com/wmp399/laser-gaze.git

cd laser-gaze
# get unitree lidar driver
echo "get unitree lidar driver"
git clone https://github.com/unitreerobotics/unilidar_sdk/tree/main
cd ..

# Get docker and install
echo "Get docker and install"
sudo apt-get install docker
sudo apt-get install docker-compose

"Build ROS-Base"
xhost local:root
cd laser-gaze/docker-noetic
docker build -t ros .

docker-compose up -d

echo "done."
echo "type `docker run -it docker-noetic_ros_1` to start the container"