---
layout: post
title: "Run Lidar in ROS 2 Docker on Raspberry Pi"
date: 2025-02-08 10:27:08 -0600
categories: Security_Camera
---
## Introduction - RPLidar and ROS

I bought an RPLidar from Amazon for my security robot's localization, mapping, and navigation. RPLidar is a 360-degree laser scanner that uses triangulation to measure distances and detect obstacles.

<a href="/assets/IMG_2918.jpeg" target="_blank">
  <img src="/assets/IMG_2918.jpeg" />
</a>

In Robotics, the Robot Operating System (ROS) is the default choice for building robot applications. It offers software libraries and tools that help developers seamlessly connect motors, sensors, and software and speed up the process of building advanced robots. 

The RPLidar has a standard ROS node that reads data from a RPLidar 2D laser scanner and publishes that data as a message to an ROS topic. This allows other ROS nodes to access and utilize the laser scan information for tasks like obstacle avoidance or mapping in robotics applications. A ROS node is a program that runs on ROS and communicates with other nodes. Nodes are the basic building blocks of ROS and are used to perform computations and control systems. 

The problem with using ROS on Raspberry Pi is that it does not support Raspbian, the OS optimized for the Raspberry Pi hardware. To use ROS on Raspberry Pi, we either install 64-bit Ubuntu or run ROS 2 Docker in the Raspberry Pi OS. I tried Ubuntu on Raspberry Pi, but it was too heavy for the little Pi. Therefore, I want to use ROS 2 Docker on Pi for my security robot. The concept of using Docker for Robotic applications is relatively new. 

By the way, ROS has two versions: ROS 1 and ROS 2. ROS 2 is the current version and is designed to be more flexible, secure, and performant than ROS 1. 

## Docker
Docker is widely used in enterprise software's microservice architecture, where each service operates as a lightweight Docker container that can be independently deployed and upgraded. A Docker container can be created from a Docker image, which serves as a template defining its structure and dependencies.

The official ROS 2 Docker images are available [here](https://hub.docker.com/_/ros/tags). There are several varieties: [ros-core, ros-base, or perception](https://www.ros.org/reps/rep-2001.html#humble-hawksbill-may-2022-may-2027), each one extending from the previous with additional ROS packages.

### Dockerfile
To build robotic applications using ROS 2 Docker, we need to create our own Dockerfile using the ROS 2 Docker image as the base image.

A Dockerfile is a text file containing instructions for creating a Docker image. It specifies the base image, adds dependencies, sets environment variables, and defines the application's startup command. In our case, the base image will be the "ros:humble-perception" ROS 2 image.

Here is a starting Dockerfile I created for my security robot. This Dockerfile creates a "ros" user, a "ros" group, establishes a home directory, and configures the environment for the user. The environment variable "ROS_DOMAIN_ID" ensures that only systems in the same domain can communicate with one another.

Additionally, this Dockerfile creates a ROS workspace, "bot_ws," alongside the user creation. This workspace is necessary for developing ROS applications, which I will discuss later.

```
FROM ros:humble-perception

# Install essential utilities
RUN apt-get update && apt-get install -y \
    vim \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# Create "ros" user and group
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV BOT_HOME=/home/$USERNAME

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME -d $BOT_HOME \
  && chown -R $USER_UID:$USER_GID ${BOT_HOME}

USER ros

RUN echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc \
  && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create the ROS 2 workspace
RUN mkdir -p ${BOT_HOME}/bot_ws/src 

# Set default working directory
WORKDIR ${BOT_HOME}/bot_ws
```
### Build and run the image
The command `docker build -t my_ros2_image .` is to create the docker image using the above "Dockerfile". I omitted the repository URL from the tag to keep the tag simple and only use the image name. 

To start the docker container: `docker run -it --rm my_ros2_image`.

## RPLidar
Now we have a starting container for the robot development. Let's see how to install the RPLidar there. The RPLidar has a USB cable that connects to the Raspberry Pi. 
![alt text](/assets/RPLidar.png)

We need to get the driver code for RPLidar to scan and publish scan data. The RPLidar has a [git repository](https://github.com/Slamtec/rplidar_ros/tree/ros2) for its ROS 2 ecosystem.  We need to clone that repository and build it using `colcon build`. The RPLidar package provides a node that publishes the scan data to the "/scan" topic, and services to control start and stop the RPLidar motor. Building from the ROS package source code is a common way to consume third-party robotic hardware.

The RPLidar uses a serial port to communicate with ROS. The port must specified when we starting the RPLidar node. To find out which USB port is used by the RPLidar, let's connect the Lidar's USB cable to the Pi and issue this command: 
```
$ ls -l /dev/serial/by-id/
lrwxrwxrwx 1 root root 13 Feb 10 17:25 usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -> ../../ttyUSB0
$ ls -l /dev/ttyUSB*
crw-rw---- 1 root dialout 188, 0 Feb 10 17:47 /dev/ttyUSB0
```
The command output tells us:
1. "/dev/ttyUSB0" is used by the RPLidar as the serial port; 
2. The root user and the "dialout" group have read-write permission on the port. 
3. The device major number is 188, which stands for "USB serial" in the [linux document](https://www.kernel.org/doc/Documentation/admin-guide/devices.txt)

## Run RPLidar in ROS 2 Docker
### Update my_ros2 Docker Image
With all this information, we are ready to update our Docker images for running RPLidar in Docker:

First, we need to add a line to the Dockerfile granting the "ros" user permission to access the ttyUSB port. 
```
# add the user to the dialout group
RUN usermod -a -G dialout ${USERNAME}
```
Next, we add the following lines to download and build the RPLidar nodes:
```
# download the rplidar source code
RUN cd ${BOT_HOME}/bot_ws/src \
  && git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# Source ROS 2 env and build the rplidar nodes
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd ${BOT_HOME}/bot_ws && \
    colcon build --symlink-install --packages-select rplidar_ros"
```
Here is the updated [Dockerfile](/code/Dockerfile).

Now, rebuild the Docker image using the updated Dockerfile with the command `docker build -t my_ros2_image .`

# Create my_ros2 Docker Container

To allow a Docker container to use a device connected to the host, we need to specify which device port we need in Docker. In the case of RPLidar, we must specify the ttyUSB port when using `docker run`. 

A straightforward approach is to use the `docker run --device` option, as follows: `docker run -it --network=host --ipc=host --device=/dev/ttyUSB0 my_ros2_image`. However, over time, I found this is not the most convenient approach, as the USB port can change after reconnecting the RPLidar's USB cable. Therefore, a better method is to map to all devices and then use device group rules to restrict it to only USB serial converters, which belong to device group 188, as mentioned in the previous section. 
```
$ docker run -it --network=host --ipc=host -v /dev:/dev --device-cgroup-rule='c 188:* rmw' my_ros2_image
```

Once we are in the Docker container, we source the install setup script and run the RPLidar ROS 2 launch file. We then list the nodes, topics, and services to ensure the RPLidar is running correctly. 

```
$ ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB0

$ ros2 node list
/rplidar_node

$ ros2 topic list
/parameter_events
/rosout
/scan

$ ros2 service list
/rplidar_node/describe_parameters
/rplidar_node/get_parameter_types
/rplidar_node/get_parameters
/rplidar_node/list_parameters
/rplidar_node/set_parameters
/rplidar_node/set_parameters_atomically
/start_motor
/stop_motor
```

If you receive the error code "80008004” from `ros2 launch rplidar_ros`, it indicates a communication issue between the ROS node and the RPLidar hardware. Please verify if the OS recognizes the LiDAR device at the specified ttyUSB. 
```
$ ls -l /dev/serial/by-id/
$ ls -l /dev/ttyUSB*
```

To stop the RPLidar motor, use the command:
```
$ ros2 service call /stop_motor std_srvs/srv/Empty
```

To start the RPLidar motor, issue the command: 
```
$ ros2 service call /start_motor std_srvs/srv/Empty
```

## Viewing the Lidar Scan

ROS 2 on the Raspberry Pi serves as the robot controller. To keep it lightweight, it has only a command interface. To monitor the robot's activities, I use a separate system, typically an Ubuntu Linux desktop, which has a graphical user interface and subscribes to the topics published by the robot.

ROS includes RViz, a 3D visualization tool for displaying sensor data and the environments in which robots operate. It assists developers and operators in observing how robots interact with their surroundings in real-time or through recorded data. RViz employs a plugin architecture that allows for the addition of custom visualizations or data displays, making it highly extensible.

As mentioned earlier, we set the environment variable "export ROS_DOMAIN_ID=0" to control who can access the data published by the robot running in ROS 2 Docker. I installed ROS 2 Desktop version on my Ubuntu Linux machine. Detailed installation steps can be found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#install-ros-2-packages).

Start RViz2 and add the "LaserScan" with the "/scan" topic. I created a [launch file](/code/view_rviz2.py) in the rplidar_ros package to streamline the steps: `ros2 launch rplidar_ros view_rviz2.py`.

Here, I can see the robot's lidar scan from my Ubuntu desktop. The left side is my office's curved bay window. A desk, a bookshelf, and a cat tower are next to the window, so it is quite busy there. 
<a href="/assets/rviz2_scan.png" target="_blank">
  <img src="/assets/rviz2_scan.png" />
</a>

