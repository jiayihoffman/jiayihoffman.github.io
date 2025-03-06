#!/bin/bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
exec bash  # Keeps the container running