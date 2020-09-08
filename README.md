# Introduction

As of September 2020, our Trash-picking Turtlebot can autonomously navigate around an unknown building, draw a map, detect & pickup water bottle, then drop into trashcan. See the videos here: https://drive.google.com/drive/folders/1VAmSW8Z5EUJZb4Y-RXb9zj3yCQSbMGp9

It requires these packages:
- Object tracking (https://github.com/Cornell-Tech-Turtlebot/object_tracking): Detect different objects including water bottle & trashcan. Localize positions of those objects on the map. Drive the robot toward those objects.
- darknet_ros (https://github.com/leggedrobotics/darknet_ros): YOLO on ROS. Detect different objects including the water bottle.
- ar_track_alvar (http://wiki.ros.org/ar_track_alvar): Detect AR tag that is sticked to the trashcan.
- Bottle manipulator (https://github.com/Cornell-Tech-Turtlebot/bottle_manipulator): Control the manipulator to pickup & drop off the water bottle.
- explore_lite (http://wiki.ros.org/explore_lite): Autonomously explore & map an unknown building.  
- Patrol (https://github.com/Cornell-Tech-Turtlebot/patrol): Randomly patrol around the mapped building.
- Orchestrator (https://github.com/Cornell-Tech-Turtlebot/orchestrator): Orchestrate other packages. For example, when `object_tracking` successfully drives the robot to approach the bottle, `orcheschator` will tell `bottle_manipulator` to start picking the bottle up.


# Setup

## Robot:
The Waffle Pi with Open Manipulator is already fully setup. In case you need to setup it again, follow this tutorial: 

- https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup
- https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation
- Calibrate the camera (important for tag detection to work): https://github.com/UbiquityRobotics/raspicam_node

## Your laptop:
Follow this to setup ROS on your laptop:
https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup
https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation

Follow this to get your laptop into the same VPN as the robot & server: 
https://github.com/travers-rhodes/gazebo_tensorflow_turtlebot_docker#connection-instructions

## Server:
The workspace `catkin_ws` on server is fully setup. It's best to leave it there or make a backup of that folder, in case we missed to document certain configuration in this documentation. It's pretty hard to config things to make the robot works smoothly!

In case you need to setup it again, follow this:

- Setup ROS on server:
        https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup
        https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation

- Install necessary packages:

        cd /catkin_ws/src
        git clone https://github.com/Cornell-Tech-Turtlebot/object_tracking.git
        git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
        git clone https://github.com/ros-perception/ar_track_alvar.git
        sudo apt install ros-${ROS_DISTRO}-multirobot-map-merge ros-${ROS_DISTRO}-explore-lite
        git clone https://github.com/Cornell-Tech-Turtlebot/bottle_manipulator.git
        git clone https://github.com/Cornell-Tech-Turtlebot/patrol.git
        git clone https://github.com/Cornell-Tech-Turtlebot/orchestrator
        
 - Build the workspace:
 
        cd /catkin_ws
        catkin build
        source devel/setup.bash

   If `catkin_build` doesn't work, try `catkin_make_isolated` then `source devel_isolated/setup.bash`
   

# How to run


# Room for improvement








