---
layout: post
title: "Droid Vision with a built-in Joystick and Keypad"
date: 2025-03-06 09:15:28 -0600
categories: Droid_Vision
---

[RPLidar in ROS 2 Docker on Raspberry Pi]: {% link _posts/2025-02-08-SecurityRobot-RPLidar.md %}

Chasing my cat and watching him run around through the lens of a mobile robot is pretty cool. However, I find it inconvenient to hold the iPad in one hand and the remote control in the other. What if I could integrate the remote control into the Droid Vision app, so that I could see and control the robot using just one device, with everything consolidated in the Droid Vision app?

This leads to Droid Vision 2.1, which is available for free from the [Apple Store](https://apps.apple.com/us/app/droid-vision/id6737351549).

# ROS and Rosbridge WebSocket
To control the robot using the Droid Vision app, I will utilize Rosbridge WebSocket.

In robotics, the Robot Operating System (ROS) is the standard choice for developing robot applications. It provides software libraries and tools that enable developers to seamlessly connect motors, sensors, and software, thereby accelerating the process of creating advanced robots. 

Rosbridge WebSocket is a communication interface that allows non-ROS applications, such as web or mobile apps, to interact with a ROS system using JSON messages over a WebSocket connection. It is part of the rosbridge_suite and enables bidirectional communication between ROS nodes and external applications without requiring them to be ROS-native. This is particularly beneficial for applications that need real-time interaction with a ROS system while avoiding the complexity of ROS 2 DDS networking.

# ROS 2 Docker Container
To run ROS on Raspberry Pi, I recommend using ROS 2 Docker. 

Docker is widely used in enterprise software's microservice architecture, where each service functions as a lightweight Docker container that can be independently deployed and upgraded. A Docker container is created from a Docker image, which serves as a template defining its structure and dependencies. 

The concept of using Docker for robotic applications is relatively new. I have a detailed article about it [RPLidar in ROS 2 Docker on Raspberry Pi].

The Dockerfile and its dependent script that I prepared for my robot can be downloaded from here: [Dockerfile](/code/Dockerfile) and [start_robot.sh](/code/start_robot.sh). We can build and start the Docker container using the following commands:
```
# build the docker image "my_bot_image"
docker build -t my_bot_image .

# start the docker container from the image
docker run -it --network=host --ipc=host -v /dev:/dev \
    --device-cgroup-rule='c 188:* rmw' \
    --device-cgroup-rule='c 166:* rmw' \
    my_bot_image
```

With that, the "rosbridge_server" is up and running on the Raspberry Pi and can be accessed from the Droid Vision app.

More screenshots of the Droid Vision configuration and live streaming will be added soon. 


