---
layout: post
title: "Upgrade LEGO Technic 42203 into a Motorized Truck" 
date: 2026-02-27 10:20:04 -0600
categories: LEGO
image: /assets/lego/IMG_4291.jpeg
---

[ROS2 Control of the LEGO Audi e-tron]: {% link _posts/2026-01-26-LEGO-ROS2_control.md %}

Want to give your LEGO Technic 42203 a fun makeover? How about turning it into a motorized construction truck that you can remotely control using a gamepad!

## Introduction

LEGO Technic 42203 is a charming construction truck with a flatbed that can carry a large load. It features doors that open to reveal the cabin's cozy interior. It’s a great choice for building a robot car. The flatbed is perfect for holding computer gear, and the flat top can be used as a lidar sensor mount. Plus, the cabin has plenty of space for a camera. 

<img src="/assets/lego/42203_boxprod_v39.png" />

## What makes a Motorized Car

To make a motorized car move, we need both a steering system and a drive system. The driving and steering gears I have are from the LEGO Technic 42160 Audi RS Q e-tron. They’re much bigger and more complex than the cute little construction truck! 

Here is the top-down, under-the-hood view of the simple construction truck:

<a href="/assets/lego/IMG_4226.jpeg" >
  <img src="/assets/lego/IMG_4226.jpeg" width="450" />
</a>

Here are the guts of the Audi rally car. I placed the truck's tires next to the monster Audi rally car's tires. How much smaller the truck's tires are! I'll design a new driving and steering system for the truck, reuse the motors and hub from the LEGO Audi, and keep the truck's form factor.

<a href="/assets/lego/IMG_4233.jpeg" >
  <img src="/assets/lego/IMG_4233.jpeg" width="350" />
</a>
<a href="/assets/lego/IMG_4238.jpeg" >
  <img src="/assets/lego/IMG_4238.jpeg" width="350" />
</a>

Now let's design the truck's steering and propulsion system. 

### Gear Rack with a Double Bevel Gear for Steering

After some research, I developed this steering system for the truck. It uses a 1x7 Gear Rack with a 12-tooth Double Bevel Gear. The Gear Rack has axle holes and pin holes that can be attached to connectors to form joints for movement. The Double Bevel Gear's axle connects to the motor. Here’s a closer look at how the connectors and pins work together. 

<a href="/assets/lego/IMG_4295.jpeg" >
  <img src="/assets/lego/IMG_4295.jpeg" />
</a>

Bevel Gears are conical gears with angled, rounded teeth designed to transfer torque at 90-degree angles.

The beige-colored axles are commonly used as the motor output. They have segments shaped like "+" to lock the axle to the motor and the gearwheel, while the smooth segment allows the axle to spin in the separator with minimal friction.

Here’s a snapshot of how it looks when it’s set up on the truck. I used the same-length front beam to keep the dimensions and shape consistent, so the upgraded truck frame can still fit under the same truck head.

<a href="/assets/lego/IMG_4260.jpeg" >
  <img src="/assets/lego/IMG_4260.jpeg" />
</a>

### Differential Gear for Propulsion
A differential is a gear mechanism that allows the left and right wheels to rotate at different speeds while still being powered by the same motor.

This is very important when a vehicle turns because the outside wheel travels a longer distance than the inside wheel. Therefore, the outside wheel must spin faster than the inside wheel. Without a differential, the wheels fight each other, and turning becomes jerky.

My LEGO Technic Differential Gear has:
* A round housing with five small 12-tooth Bevel Gears inside and a 28-tooth bright yellow Bevel Gear on the outside as the "lid"
* One input gear in the center, which is a 14-tooth bright yellow Bevel Gear that transfers motion at a 90-degree angle
* Two axle outputs for the wheels

<a href="/assets/lego/IMG_4297.jpeg" >
  <img src="/assets/lego/IMG_4297.jpeg" />
</a>

In my truck, the motor sits a bit higher than the differential's input. To transfer the rotation downward, I vertically aligned two spur gears on the motor's output shaft. A spur gear is a flat gear with straight teeth.

### Putting All Together

Here’s the base frame for the upgraded truck! I removed the original gear set that lifts and lowers the cargo bed and used the space for the two motors. 

<a href="/assets/lego/IMG_4262.jpeg" >
  <img src="/assets/lego/IMG_4262.jpeg" />
</a>

Here is the truck's original base frame for reference:

<a href="/assets/lego/IMG_4222.jpeg" >
  <img src="/assets/lego/IMG_4222.jpeg" />
</a>

## Show Time
Here is how the upgraded truck looks. You won't notice much difference at first glance, except for the power supply in the trunk. Keeping the same look is my goal. :)

<a href="/assets/lego/IMG_4291.jpeg" >
  <img src="/assets/lego/IMG_4291.jpeg" />
</a>

Enjoy a quick demo of me driving the construction truck with ROS2 Control using a gamepad!

For information on how to control a LEGO motorized car using a gamepad, please checkout my earlier post on [ROS2 Control of the LEGO Audi e-tron].

<iframe width="800" height="468"
src="https://www.youtube.com/embed/qxccaJngeqI?autoplay=1&mute=0">
</iframe>

Cheers!
