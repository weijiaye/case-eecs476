#!/bin/bash

# This script will automatically configure ROS networking for one of the 
#   Kobuki robots used in the Glennan 210 lab. We choose the "02".
# Written Feb 23, 2017 by Trent Ziemer

# Get the current IP address used by the remote desktop computer (not the robot)
address=$(ip route get 8.8.8.8 | awk '{print $NF; exit}')

echo "Setting var ROS_IP = $address, which is your IP address"
export ROS_IP=$address

# Set the remote robot systems IP (totally static)
master_address="http://deeplearning03w.eecs.cwru.edu:11311"
echo "Setting var ROS_MASTER_URI = $master_address, which is the Kobuki robot IP"
export ROS_MASTER_URI=$master_address
# Gotta 'host', right?

echo "Now, do you need to 'host' something?"

