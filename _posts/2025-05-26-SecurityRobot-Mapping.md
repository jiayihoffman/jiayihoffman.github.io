---
layout: post
title: "Robot Auto Mapping using Nav2 SLAM Toolbox"
date: 2025-05-26 10:27:08 -0600
categories: Security_Robot
---

I am always curious about how Roomba automatically creates a floor map of the house by driving around. In this article, I will explain how to create the floor map using a mobile robot with Nav2 SLAM Toolbox. 

## Get to know the SLAM Toolbox

The Nav2 SLAM Toolbox is a package in ROS 2 (Robot Operating System) used for Simultaneous Localization and Mapping (SLAM). It enables a robot to create the map of an unknown environment while tracking its position on that map in real time. The map created is a 2D occupancy grid map for navigation later on.

Here is the kitchen map created by the robot while it was circling the kitchen. 

<a href="/assets/slam/mapping.png" target="_blank">
  <img src="/assets/slam/mapping.png" />
</a>

<iframe width="800" height="468"
src="https://youtube.com/embed/U6OvrCttuMQ?autoplay=1&mute=0">
</iframe>

To be continued ...