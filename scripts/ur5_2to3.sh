#!/usr/bin/env bash
#packages=(moveit moveit_msgs moveit_resources moveit_visual_tools \
#          sawyer_moveit srdfdom intera_sdk sawyer_simulator)
packages=(moveit moveit_msgs moveit_resources moveit_visual_tools)
#ros_ws=<ROS_WS>
ros_ws=~/mujoco_ros/src/
for package in ${packages[@]}; do
    python_scripts=`find "${ros_ws}${package}" -name "*.py"`
    for script in ${python_scripts[@]}; do
        2to3 -w ${script}
    done
done
