---
layout: post
title: "Droid Vision Step-by-Step Tutorial"
date: 2024-11-24 10:53:28 -0600
categories: Droid_Vision
---
# See the world through a robot’s eyes 

<iframe width="1000" height="575"
src="https://www.youtube.com/embed/UXnyetTAiyk?autoplay=0&mute=0">
</iframe>


### The Robot
To use the Droid Vision app, we first need to configure the robot, a Raspberry Pi with a Pi Camera, on an RC car. The above video shows how to set up the camera and the streaming service on the robot. Here is a quick summary of the steps: 

[blog post]: {% link _posts/2024-11-18-DroidVision-RC.md %}

1. Install the Pi Camera on the Raspberry Pi. Run `rpicam-hello` command to ensure the Pi camera is working correctly.
2. Install GStreamer libraries on the Raspberry Pi. Library details are in another [blog post], section "Setup GStreamer on the Raspberry Pi". <br/>Once libraries are installed, verify the GStreamer installation using the `gst-launch-1.0 --version` commands.
3. Find the Raspberry Pi's IP address.
4. Start the RTSP service on Raspberry Pi using `python rtsp_server.py`. Note down its RTSP URL.

Once the streaming service is running, we can connect to it using the Droid Vision app. 

### Droid Vision App
To connect to the droid’s RTSP service, tap the "+" button in the App and enter the robot’s name and RTSP URL captured in the previous step. Save it and tap "Go Live View" to view and record the live video. 

The events in the "Live View History" can be removed by left-swiping the row. 

### Get the Droid Vision app
Download it for free from the [App Store](https://apps.apple.com/us/app/droid-vision/id6737351549) in Apple.
