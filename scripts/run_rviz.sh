#!/bin/sh

# WOROKAROUND FOR RVIZ ISSUE: 
# rviz RobotModel broken in melodic: <visual><origin> not used #1249  
# Script sets numeric locale for utf-8

export LC_ALL="en_US.UTF-8"
rosrun rviz rviz
