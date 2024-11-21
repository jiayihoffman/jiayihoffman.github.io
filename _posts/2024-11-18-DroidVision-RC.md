---
layout: post
title: "Droid Vision on an RC Truck"
date: 2024-11-18 23:53:28 -0600
categories: Droid_Vision
---
## Introduction
I love robots, AI, and racing. I believe the future world is filled with robots that live with us. I am always curious about if we can capture what the robot sees. Therefore, I created the app "Droid Vision". It is available in the App Store. This app fetches the video data from the RTSP service running on the robot. 

In this blog, I will show you how to install the RTSP streaming service on a Radio-controlled truck and use the Droid Vision app to create videos like these:

<video width="640" height="480" preload="auto" muted controls>
  <source src="/assets/robot_truck.mp4" type="video/mp4">
</video>

<video width="640" height="480" preload="auto" muted controls>
  <source src="/assets/cat.mp4" type="video/mp4">
</video>

## Radio-controlled car

The RTSP streaming service will be installed on a Raspberry Pi with the Pi Camera Module. Therefore, the first step is to find an RC car with space for the Raspberry Pi and camera. 

In a local hobby shop, I found one like this. It is called the "Mojave Grom 4X4 Desert Truck" from Arrma RC. The truck is about one foot long, half a foot wide, and a half foot tall. It is a good size to fit in the Raspberry Pi. The truck has a radio receiver controlling the wheel's rotation and turning. Besides the truck, the set comes with a Spektrum Radio control, a LiPO battery, and a USB charger. The truck is ready to run once the battery is charged. The whole set costs $149.99, which I think is reasonably priced. 

This RC truck is pretty durable. I occasionally drove it into a wall or a curb, but the front and back bumpers absorbed the impact, so the frame remained unharmed. 

![RC Car](/assets/IMG_2725.jpeg)

## Upgrade the RC truck with a Camera

Now, it is time to upgrade the RC truck. The truck's removable outer shell is handy when I modify it. I removed the shell, removed the "drivers," and replaced them with the Raspberry Pi and the Pi camera.

![RC_PI_1](/assets/IMG_2724.jpeg)

![RC_PI_2](/assets/IMG_2675.jpeg)

### Raspberry Pi

I am using Raspberry Pi 5 with the Pi Camera Module 3. Raspberry Pi 5 is the latest generation of Pi with 2 to 3x the speed of the previous generation. The Camera Module 3 is also new, released in 2023, after seven years of long-awaited updates to the Camera 2. The Raspberry Pi and its camera can be purchased from Amazon or the PiShop. Raspberry Pi 5 camera port is 15 Pin, so be sure to get the ribbon cable that has 15 Pin on one end and 22 Pin on the other end. 

The Raspberry Pi OS I installed is Debian version 12, nicknamed Bookworm. The OS is installed on the mini SD card using "Raspberry Pi Imager". In the Imager, I chose "Raspberry Pi 5" as the "Raspberry Pi Device" option and "64-bit Raspberry Pi Desktop" for the "Operating System" drop-down. 

### Setup Pi Camera on the Raspberry Pi

Connecting the Pi camera to the Raspberry Pi using the ribbon cable is standard, and there are lots of videos online showing how to connect them, so I am not repeating it here. 

Once the two are connected, please start Raspberry Pi and run `rpicam-hello` command in the terminal to verify the connection is good. You can get to the Raspberry Pi's terminal either through SSH or by connecting it with a monitor, keyboard, and mouse. 

* If you are using SSH and see these printouts from the "rpicam-hello" command, the connection has been successful. 
```
#0 (0.00 fps) exp 32680.00 ag 5.39 dg 1.37
#1 (30.01 fps) exp 32680.00 ag 5.39 dg 1.37
#2 (30.02 fps) exp 32680.00 ag 7.26 dg 1.01
...
```

* If you are in front of the Raspberry Pi monitor and see the camera preview stream displayed on the screen, it means the camera to Raspberry Pi's connection is good. 

If the camera cannot stream video, please check the ribbon cable connection and see if it faces the right direction. 

## Video Streaming

Now we have the camera configured to work with the Raspberry Pi, we are ready for video streaming. With video streaming, there are two popular options: UV4L and GStreamer. I tried both, and here are my observations:

**UV4L** is explicitly designed for Raspberry Pi and has a smaller user base. It is simpler to set up on the older version of Raspberry Pi for basic WebRTC streaming. I used UV4L on Raspberry Pi OS "Buster" for WebRTC streaming. However, UV4L does not work with the latest Raspberry Pi OS, "Bookworm." Installing UV4L on the latest Raspberry Pi OS leads to packaging errors. 

**GStreamer** is highly flexible. Construct pipelines give the user precise control over video processing, encoding, and transmission. Therefore, GStreamer typically outperforms UV4L in complex video streaming and processing scenarios. GStreamer supports a wide range of protocols, has a large, active community, and is widely used across different platforms.

Therefore, the choice is obvious. We will use GStreamer to stream video from the RC truck. 

### Setup GStreamer on the Raspberry Pi

The Raspberry Pi Bookworm comes preinstalled with Python 3.11. I will provide a Python script that constructs and runs a GStreamer pipeline to stream video using RTSP protocol. 
Before we can run that script, we need to install GStreamer plugins and dependent libraries.

```
# install gstreamer tools and core modules
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
sudo apt install gstreamer1.0-plugins-base-apps v4l-utils

# install libcamera bits
sudo apt install libcamera-dev gstreamer1.0-libcamera

# install rtsp related libs
sudo apt install libgstrtspserver-1.0-dev
sudo apt install gstreamer1.0-rtsp
sudo apt install -y python3-gi python3-gi-cairo gir1.2-gtk-3.0 gir1.2-gst-rtsp-server-1.0
```

### Start the RTSP Streaming Service on Raspberry Pi

SSH into the Raspberry Pi (`ssh pi@<your_raspberry_pi_ipaddress>`) and create a folder called "streamer" in the pi user's home directory. Download the Python script [rtsp_server.py](/code/rtsp_server.py) and place it in the "streamer" folder. 

To run the RTSP server on the RC truck, type the command `python rtsp_server.py` on the Raspberry Pi, for example:
```
$ cd streamer
$ python rtsp_server.py
RTSP server is running on rtsp://localhost:8554/stream
```

### Use the Droid Vision App

To view the RC truck's video stream in the Droid Vision app, open the app on the iPhone or iPad, add the robot info by giving it a name, and setting its RTSP URL, which is:
 
`rtsp://<your_raspberry_pi_ipaddress>:8554/stream`

<img src="/assets/IMG_2726.PNG" width="350" />  <img src="/assets/IMG_2727.PNG" width="350" />

In the app, tap "Go Live View", then you will see the truck's live view on your iPhone/iPad, and the RTSP server terminal window will print out the following messages:
```
[0:14:25.171703990] [2040]  INFO Camera camera_manager.cpp:316 libcamera v0.3.1+74-a65f44f4-dirty (2024-09-09T08:55:36BST)
[0:14:25.181487052] [2041]  INFO RPI pisp.cpp:695 libpisp version v1.0.7 28196ed6edcf 29-08-2024 (16:33:32)
[0:14:25.195305488] [2041]  INFO RPI pisp.cpp:1155 Registered camera /base/axi/pcie@120000/rp1/i2c@80000/imx708@1a to CFE device /dev/media0 and ISP device /dev/media1 using PiSP variant BCM2712_C0
[0:14:25.198211593] [2044]  INFO Camera camera.cpp:1191 configuring streams: (0) 640x480-YUYV
[0:14:25.198383912] [2041]  INFO RPI pisp.cpp:1451 Sensor: /base/axi/pcie@120000/rp1/i2c@80000/imx708@1a - Selected sensor format: 1536x864-SBGGR10_1X10 - Selected CFE format: 1536x864-PC1B
...
```