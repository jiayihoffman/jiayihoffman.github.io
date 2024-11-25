---
layout: post
title: "Droid Vision Step-by-Step Video Tutorial"
date: 2024-11-24 10:53:28 -0600
categories: Droid_Vision
---
# See the world through a robot’s eyes 

[![Video Tutorial](https://img.youtube.com/vi/UXnyetTAiyk/0.jpg)](https://www.youtube.com/watch?v=UXnyetTAiyk)

### The Robot
To use the Droid Vision app, we first need to configure the robot, a Raspberry Pi with a Pi Camera, on an RC car. The video shows how to set up the camera and the streaming service on the robot. Here is a quick summary of the steps:

1. Install the Pi Camera on the Raspberry Pi. Run `rpicam-hello` command to ensure the Pi camera is working correctly.
3. Install GStreamer libraries on the Raspberry Pi. Details are in the [blog post](2024-11-18-DroidVision-RC.md#setup-gstreamer-on-the-raspberry-pi). Verify the GStreamer installation using the `gst-launch-1.0 --version` command.
3. Find the Raspberry Pi's IP address.
4. Start the RTSP service on Raspberry Pi using `python rtsp_server.py`. Note down its RTSP URL.

Once the streaming service is running, we can connect to it using the Droid Vision app. 

### Droid Vision App
To connect to the RTSP service on the bot, tap the "+" button in the App and enter the name and RTSP URL captured in the previous step. 

The events in the "Live View History" can be removed by left-swiping the row. 
