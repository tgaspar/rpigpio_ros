#!/bin/bash
# PATH=$PATH:/usr/local/bin

source /home/ubuntu/.bashrc
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

echo $(/opt/ros/kinetic/bin/rospack find rpigpio_ros)
# /bin/echo $PATH
export ROS_MASTER_URI=http://localhost:11311/
/opt/ros/kinetic/bin/rosrun rpigpio_ros gpio_control.py -c $(/opt/ros/kinetic/bin/rospack find rpigpio_ros)/conf/example.yml