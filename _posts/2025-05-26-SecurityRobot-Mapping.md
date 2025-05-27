---
layout: post
title: "Robot Auto Mapping using Nav2 SLAM Toolbox"
date: 2025-05-26 10:27:08 -0600
categories: Security_Robot
---

[RPLidar in ROS 2 Docker on Raspberry Pi]: {% link _posts/2025-02-08-SecurityRobot-RPLidar.md %}

I am always curious about how the Roomba vacuum automatically creates a map of the floor by driving around. In this article, I will explain how to create the floor map using a mobile robot with the Nav2 SLAM Toolbox.

## Get to Know the SLAM Toolbox

The Nav2 (Navigation 2) stack is a modular system that enables autonomous navigation for mobile robots in ROS. The SLAM Toolbox, which stands for Simultaneously Localize and Map, plays a key role in the navigation stack and is part of ROS Nav2.

### Mapping 

The SLAM Toolbox is a 2D mapping system based on LiDAR data and odometry. It enables the robot to generate a map of an unknown environment while tracking its position on that map in real time. SLAM Toolbox builds an 2D occupancy grid map using the robot's laser scans and motion data. 

Here is the kitchen map made by my robot R4 as it circled the kitchen. 

<a href="/assets/slam/mapping.png" target="_blank">
  <img src="/assets/slam/mapping.png" />
</a>

<iframe width="800" height="468"
src="https://youtube.com/embed/U6OvrCttuMQ?autoplay=1&mute=0">
</iframe>

The tool that displays the map is RViz2, which is a 3D visualization tool within the ROS 2 framework. RViz2 enables users to view and interact with a robot's state, sensor data, and environment in a 3D space. It provides a window into the robot's world, illustrating what the robot "sees" and how it is positioned.

The generated map can be saved in RViz2 using the "slam_toolbox" panel. This panel contains the "save" and "serialization" options, where the “serialization” format is intended for use by the SLAM Toolbox itself, while the "save" format allows the map to be utilized by other Nav2 localization tools, such as AMCL.

Mapping is the first step of the navigation pipeline, and SLAM Toolbox plays a crucial role in map generation. 

### Localization

The SLAM Toolbox is also a localization system that identifies the robot's position and orientation (pose) within its perceived world. 

To explain the concept of "location" in ROS, we need to discuss the ROS Frame and Transform.

#### Frame 
* **Frame** refers to a 3D coordinate system used to define the spatial location and orientation of an entity (like a robot, sensor, or object) in space.
* **odom Frame** is the starting point of the robot’s trajectory. As the robot moves, its pose changes in the odom frame based on odometry data.
* **base_link Frame** is a fixed frame attached to the robot's chassis or main body. It serves as a reference point for other robot frames, such as wheels and lidar sensors.

Here are the frames of my mobile robot, as seen in RViz, with red representing x axies, green representing y, and blue representing z. Every joint defined in the robot's URDF has a frame.

<a href="/assets/slam/rviz2.png" target="_blank">
  <img src="/assets/slam/rviz2.png" />
</a>


#### Transform chain
What is Transform in ROS? 

* **Transform(TF)**, represents the position and orientation of a coordinate frame relative to another. It's a fundamental concept in ROS to describe how objects and coordinate systems are located in 3D space, enabling systems to understand the relative positions of various components and their interactions. 

A typical Transform chain for mobile robots looks like this:

```
map → odom → base_link → laser_frame
```

Each transform has a purpose:
* **odom → base_link** comes from the wheel encoder or other local sensors. It tracks how far the robot has moved from its starting point. It is fast to compute, continuous but accumulates drifts over time. <br><br>To correct for odometry drift, a higher-level system like SLAM Toolbox computes the robot’s true global pose and broadcasts.

* **map → odom** comes from the SLAM Toolbox and provides global correction to the drifting odometry. <br><br>The SLAM Toolbox estimates the robot’s pose in the global map frame based on the Laser scan and odom data. It uses this information to compute a correction transform, and broadcasts it as map → odom. This allows the robot’s current global position to be determined as base_link in the map frame.

The "map → odom" transform is essential for: 
1. RViz visualization, so the robot appears in the correct position on the map.
2. Path planning, so global planners can compute paths from the robot’s actual location.
3. Localization, so the robot can recognize when it returns to a previously visited place (loop closure).

In summary, SLAM Toolbox broadcasts map → odom to anchor the robot’s local odometry into the global map frame. This corrects for drift and ensures accurate global navigation. Without this transform, Nav2 cannot correctly localize the robot or plan valid paths. 

## Meet Robot R4
This is my Differential Mobile Robot featuring two independently driven wheels and two caster wheels for balance. The Differential Bot moves forward or backward by rotating both wheels in the same direction, and it can rotate in place by spinning the wheels in opposite directions.

At the top of the robot is the RPLidar, which generates the scan data. I have a blog [RPLidar in ROS 2 Docker on Raspberry Pi] that details how to configure and run the Lidar nodes on a real robot. Please check that out.

<a href="/assets/teleop/IMG_3220.jpeg" target="_blank">
  <img src="/assets/teleop/IMG_3220.jpeg" width="400" />
</a>

## Run the Robot with Mapping 

To run the robot that generates maps, we need a ROS environment that includes the robot modules, the RPLidar driver, the navigation stack, the rosbridge suite, and many other dependencies. The cleanest and simplest way is to use a Docker container. Docker simplifies deployment and makes the environment portable, consistent, and shareable.

### A Docker Container
A Docker container is created from a Docker image, which serves as a template defining its structure and dependencies. Docker is widely used in enterprise software's microservice architecture but is relatively new in robotic applications. I have a blog [RPLidar in ROS 2 Docker on Raspberry Pi] explaining in detail how to use a Docker container in a robot. 

The Dockerfile I prepared for my robot can be downloaded from here: [Dockerfile](/code/Dockerfile) and [requirements.txt](/code/requirements.txt). The Docker image created from the Dockerfile includes all the necessary ROS 2 modules, Python libraries, and source code repositories. Access to the private Git repository is granted using an SSL key.

We can build the Docker image and start the Docker container using the following commands. Building the Docker image compiles the source code and creates the bot environment.

```
# build the docker image "my_bot_image"
docker build -t my_bot_image .

# start the docker container from the image
docker run -it --rm --network=host --ipc=host -v /dev:/dev \
    --device-cgroup-rule='c 188:* rmw' \
    --device-cgroup-rule='c 166:* rmw' \
    my_bot_image

```

### Commands to Run the Robot with Mapping  

On the Raspberry Pi Docker container, we launch the robot, the SLAM toolbox, and the navigation server. Additionally, I start the rosbridge_server because I use Droid Vision to view and control the robot remotely from my phone. If you prefer the traditional ROS teleop_twist_keyboard or teleop_twist_joy for remote control, feel free to use that instead and skip running the rosbridge_server.  

```
# 1. Start the robot itself
ros2 launch my_bot robot.launch.py

# 2. Start the robot's LiDAR scan
ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser_frame

# 3. Start the slam toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

# 4. Start the navigation server
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false

# 5. Start the rosbridge_server for Droid Vision Teleop
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

On the Linux development machine, launch RViz to visualize the robot in its perceived world.

```
rviz2 -d ~/dev/dev_ws/src/my_bot/config/view_bot_map.rviz
```

<a href="/assets/slam/mapping.png" target="_blank">
  <img src="/assets/slam/mapping.png" />
</a>

Cheers! :D