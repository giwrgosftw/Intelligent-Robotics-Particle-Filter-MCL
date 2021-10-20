#!/bin/bash

gnome-terminal -x bash -c "roscore" xdotool getactivewindow windowminimize
sleep 4
gnome-terminal -x bash -c "rosrun map_server map_server '/home/'$USER'/catkin_ws/src/pf_localisation/data/real_data/map.yaml'" xdotool getactivewindow windowminimize
sleep 4
gnome-terminal -x bash -c "rosrun stage_ros stageros '/home/'$USER'/catkin_ws/src/socspioneer/data/meeting.world'" xdotool getactivewindow windowminimize
sleep 4
gnome-terminal -x bash -c "roslaunch socspioneer keyboard_teleop.launch xdotool" getactivewindow windowminimize
