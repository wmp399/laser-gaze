# check for updates and apply them
echo "check for updates and apply them"
sudo apt update
sudo apt upgrade -y

# get laser-gaze files
echo "get laser-gaze files"
git clone https://github.com/wmp399/laser-gaze.git

# Remove any existing docker installation
echo "Remove any existing docker installation"
sudo apt-get purge docker-ce docker-ce-cli containerd.io -y

# Get docker and install
echo "Get docker and install"
sudo apt-get install docker

"Build ROS-Base"
cd laser-gaze/docker
docker build -t ros2 .

echo "done."
# echo "type `docker run -it -v /home/geeks/laser-gaze:/ros2 ros2` to run ros2"