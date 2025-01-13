#!/bin/bash

docker exec airbot pkill -SIGINT -f "roslaunch ros_interface airbot_arm.launch"
docker exec airbot pkill -SIGINT -f "ros2 run ros1_bridge dynamic_bridge"
# sleep 3
docker stop airbot