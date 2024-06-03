# LaserGaze
The LaserGaze system takes in data from a 4D LIDAR Scanner and provides auditory feedback to assist persons with visual impairments drive a vehicle. 

## Deployment
This program is intended to run on a Raspberry Pi 5, but to help accomodate as many potential deployment options as possible the system is contained within a docker container. Thus, any computer with USB host capabilities and the ability to run docker should be able to run the LaserGaze system. 

## Dependencies
Running LaserGaze requires a few depedencies - docker, docker-compose, the Unitree Robotics SDK, and the Unitree Robotics Point-LIO Example. Running the script setup_pi.sh will install these dependencies automatically. 

## Building and Starting the Docker Image
Running the script setup_pi.sh will install these dependencies automatically. To build the docker image manually, you can run the following terminal commands. Note the line that says `xhost local:root`. This allows any graphics inside the docker container to be piped out to the host via X11. This does not need to be run if data visualization is not needed or run on a seperate host.

```console
xhost local:root
cd laser-gaze/docker-noetic
docker build -t ros .
docker-compose up -d
docker exec -it docker-noetic_ros_1 bash
```

## Starting LaserGaze


## Useful Links / References: 
ROS Documentation: https://wiki.ros.org/
Unitree Lidar L1 SDK: https://github.com/unitreerobotics/unilidar_sdk/tree/main
Unitree LIO Implementation: https://github.com/unitreerobotics/point_lio_unilidar/tree/main
Learn ROS Course by Kevin McAleer (including getting ROS running in Docker): https://www.kevsrobots.com/learn/learn_ros/