#!/bin/bash

gnome-terminal -x bash -c "roscore" xdotool getactivewindow windowminimize
sleep 4
gnome-terminal -x bash -c "rosrun map_server map_server '/home/'$USER'/catkin_ws/src/pf_localisation/data/sim_data/meeting.yaml'" xdotool getactivewindow windowminimize
sleep 4
gnome-terminal -x bash -c "rosrun stage_ros stageros '/home/'$USER'/catkin_ws/src/socspioneer/data/meeting.world'" xdotool getactivewindow windowminimize
sleep 4
gnome-terminal -x bash -c "rosrun tf static_transform_publisher 0 0 0 0 0 0 1 /map /odom 100" xdotool getactivewindow windowminimize
sleep 4
gnome-terminal -x bash -c "rosrun rviz rviz -d /home/'$USER'/catkin_ws/src/rviz/run.rviz" xdotool getactivewindow windowminimize
sleep 4
gnome-terminal -x bash -c "roslaunch socspioneer keyboard_teleop.launch" xdotool getactivewindow windowminimize
