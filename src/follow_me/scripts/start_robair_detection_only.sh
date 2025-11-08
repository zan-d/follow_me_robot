#!/bin/bash
echo " # # # # #                          # # # # # "
echo " # # #          [ R O B A I R ]         # # # "
echo " # # # # #                          # # # # # "
echo " "
#Warning for run location because of hardcoded rviz config file path.
echo "! WARNING:  Make sure you are running this script from the ~/ros2_ws/src/follow_me/scripts folder"

#This warning is not necessary if this line is added to the .bashrc file.
#echo "! WARNING:  Make sure you have sourced the ROS install before running this script: source /opt/ros/jazzy/setup.bash"

#This Warning is to make sure they have sourced the workspace in order to launch the ros nodes. Shouldn't be needed if the script is run from the right location.
#echo "! WARNING:  Make sure you have sourced the local ROS overlay before running this script: source install/local_setup.bash"

echo " "
echo "[RobAIR] launching separate terminals for each node..."
echo " "

source ../../../install/local_setup.bash


xterm -fa monaco -fs 13 -bg black -fg white -e "ros2 run rviz2 rviz2 -d ../config/laser_only.rviz;/bin/bash" & 

xterm -fa monaco -fs 13 -bg black -fg white -e "ros2 run action robot_moving_node;/bin/bash" &
xterm -fa monaco -fs 13 -bg black -fg white -e "ros2 run follow_me detection_node;/bin/bash" &
xterm -fa monaco -fs 13 -bg red -fg white -e "ros2 run smooth_teleop smooth_teleop_node;/bin/bash" &


echo "[RobAIR] Robot is running. Press [ctrl+c] in this terminal to kill all nodes"

input=""
while [ 1 ]
do
    read input
done
