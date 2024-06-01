# LaserGaze
The LaserGaze system takes in data from a 4D LIDAR Scanner and provides auditory feedback to assist persons with visual impairments drive a vehicle. 

# Deployment
This program is intended to run on a Raspberry Pi 5, but to help accomodate as many potential deployment options as possible the system is contained within a docker container. Thus, any computer with USB host capabilities and the ability to run docker should be able to run the LaserGaze system. 

# Dependencies
Running LaserGaze requires a few depedencies - docker, docker-compose, the Unitree Robotics SDK, and the Unitree Robotics Point-LIO Example. Running the script setup_pi.sh will install these dependencies automatically. 

# Building and Starting the Docker Image
Running the script setup_pi.sh will install these dependencies automatically. To build the docker image manually, you can run: 

```console
cd laser-gaze/docker-noetic
docker build -t ros .
docker-compose up -d
docker exec -it docker-noetic_ros_1 bash
```

# Starting LaserGaze


# Useful Links / References: 

https://www.kevsrobots.com/learn/learn_ros/
https://github.com/unitreerobotics/unilidar_sdk/tree/main
https://github.com/unitreerobotics/point_lio_unilidar/tree/main