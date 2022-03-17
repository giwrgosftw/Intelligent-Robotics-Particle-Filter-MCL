# Intelligent Robotics - Particle Filter Localization (MCL)
**School of Computer Science - University of Birmingham 2021-2022**

**Authors: Haider Abbasi, Georgios Karanasios and Huw Ranscombe-Smith**

The objective of this exercise was to implement, analyze, and understand the working of a version of the Particle Filter Localization Algorithm (called Monte Carlo Localization, MCL). The project was implemented through a simulation, specifically using the ROS-Stage and Rviz platforms and it was written in Python programming language.

In order to write the localisation node, we used the skeleton code provided https://github.com/JonFreer/pf_localisation.
Then, we used the map provided for the practice exercise in the "socspioneer" package. In addition to the steps of the Particle Filtering (i.e., motion update, sensor update, and re-sampling), we also investigated methods for obtaining a single estimate of the robot’s pose at each time step and we dealt with the ”kidnapped robot” problem.

The rosbag contains data published on the following topics as the robot was being teleoperated in the simulated world:
* /base_pose_ground_truth
* /odom
* /tf
* /base_scan

## How to run (tested on Linux Ubuntu 20.04.3)
1. Read the README.md file in the following repository link for further instructions on how to compile and use the package (https://github.com/JonFreer/pf_localisation)
2. After the above step, you should also have the "socspioneer" folder in your "/catkin_ws/src/" space 
3. Download the GitHub repository files and add/replace the downloaded "pf_localisation" and "rviz" folders to the "/catkin_ws/src/" folder
4. Place the "run.sh" file on your desktop and run it through your terminal (check README.md inside the "script" folder for further instructions)

## Demo video link
https://youtu.be/Eb6K9apxAm0
