---
layout: post
title: "Droid Vision on an RC Car"
date: 2024-11-18 23:53:28 -0600
categories: Droid_Vision
---
## Introduction
I love robots, AI, and racing. I believe the future world is filled with robots that live with us. I am always curious about if we can capture what the robot sees. Therefore, I created the app "Droid Vision". It is available in the App Store. This app fetches the video data from the RTSP service running on the robot. 

In this blog, I will show you how to install the RTSP streaming service on a Radio-controlled car and use the Droid Vision app to create videos like these:

<video width="640" height="480" preload="auto" muted controls>
  <source src="/assets/robot_truck.mp4" type="video/mp4">
</video>

<video width="640" height="480" preload="auto" muted controls>
  <source src="/assets/cat.mp4" type="video/mp4">
</video>

## Radio-controlled Car

The RTSP streaming service will be installed on a Raspberry Pi with the Pi Camera Module. Therefore, the first step is to find an RC car with space for the Raspberry Pi and camera. 

In a local hobby shop, I found one like this. It is called the "Mojave Grom 4X4 Desert Truck" from Arrma RC. The truck is about one foot long, half a foot wide, and a half foot tall. It is a good size to fit in the Raspberry Pi. The truck has a radio receiver controlling the wheel's rotation and turning. Besides the truck, the set comes with a Spektrum Radio control, a LiPO battery, and a USB charger. The truck is ready to run once the battery is charged. The whole set costs $149.99, which I think is reasonably priced. 

This RC truck is pretty durable. I occasionally drove it into a wall or a curb, but the front and back bumpers absorbed the impact, so the frame remained unharmed. 

![RC Car](/assets/IMG_2725.jpeg)

## Upgrade the RC with a Camera

Now, it is time to upgrade the RC Truck. The truck's removable outer shell is handy when I modify it. I removed the shell, removed the "drivers," and replaced them with the Raspberry Pi and the Pi camera.

![RC_PI_1](/assets/IMG_2724.jpeg)

![RC_PI_2](/assets/IMG_2675.jpeg)

### Raspberry Pi

I am using Raspberry Pi 5 with the Pi Camera Module 3. Raspberry Pi 5 is the latest generation of Pi with 2 to 3x the speed of the previous generation. The Camera Module 3 is also new, released in 2023, after seven years of long-awaited updates to the Camera 2. The Raspberry Pi and its camera can be purchased from Amazon or the PiShop. Raspberry Pi 5 camera port is 15 Pin, so be sure to get the ribbon Cable that has 15 Pin on one end and 22 Pin on the other end. 

The Raspberry Pi OS I installed is Debian version 12, nicknamed Bookworm. The OS is installed on the mini SD card using "Raspberry Pi Imager". In the Imager, I chose "Raspberry Pi 5" as the "Raspberry Pi Device" option and "64-bit Raspberry Pi Desktop" for the "Operating System" drop-down. 

### Setup Pi Camera on the Raspberry Pi

Connecting the Pi camera to the Raspberry Pi using the ribbon cable is standard, and there are lots of videos online showing how to connect them, so I am not repeating it here. 

Once the two are connected, please start Raspberry Pi and run `rpicam-hello` command in the terminal to verify the connection is good. You can get to the Raspberry Pi's terminal either through ssh or by connecting it with a monitor, keyboard, and mouse. 

* If you are using ssh and see these printouts from the "rpicam-hello" command, it means the connection is successful. 
```
#0 (0.00 fps) exp 32680.00 ag 5.39 dg 1.37
#1 (30.01 fps) exp 32680.00 ag 5.39 dg 1.37
#2 (30.02 fps) exp 32680.00 ag 7.26 dg 1.01
...
```

* If you are in front of the Raspberry Pi monitor and see the camera preview stream displayed on the screen, it means the camera to Raspberry Pi's connection is good. 

If the camera cannot stream video, please check the ribbon cable connection and see if it faces the right direction. 

### Video Streaming

Now we have the camera configured to work with the Raspberry Pi, the next step is video streaming.  