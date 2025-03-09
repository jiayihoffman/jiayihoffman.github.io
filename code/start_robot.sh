#!/bin/bash

# launch my robot
# ros2 launch my_bot robot.launch.py &

# start the rosbridge websocket
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
