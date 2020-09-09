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

Follow this guide step-by-step to replicate what you see in the videos above.

# Setup

## Robot:
The robots are already fully setup. The passwords to SSH into the robots:

- Waffle Pi: `admin`
- Waffle Pi with Open Manipulator: `turtlebot`

In case you need to setup the robots again, follow this tutorial: 

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
The workspace `/catkin_ws` on server is fully setup. It's best to just use it, or make a backup of that folder if you decide to create a new workspace or virtual machine. There might be certain small configs we made to get things working, and we may missed it in this documentation. It's pretty hard to config things to make the server & robot work smoothly!

In case you need to setup it again, follow this. Again, this may not include some small configs we made:

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
   

# Bring up basic packages
## On Robot:
Run each command below in a separate Terminal window:

        roslaunch turtlebot3_bringup turtlebot3_robot.launch
        roslaunch turtlebot3_bringup turtlebot3_rpicamera_2.launch    

## On Server:
Run each command below in a separate Terminal window:
        
        rosrun image_transport republish compressed in:=/raspicam_node/image out:=/raspicam_node/image_raw
        rosrun tf static_transform_publisher 0 0 0 0 0 0 camera_rgb_optical_frame raspicam 100
        roslaunch object_tracking turtlebot3_slam.launch
        roslaunch turtlebot3_navigation move_base.launch
        roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
        roslaunch turtlebot3_manipulation_moveit_config move_group.launch
        vglrun rviz



# Autonomous mapping of an unknown building
On **Server**, run this command:
        roslaunch explore_lite explore.launch

The robot will automatically explore & map unknown areas of building,  until there is no unknown area left.

## On Server:
Run each command in a separate Terminal window to bring up the necessary packages:
        
        
        rosrun bottle_manipulation execute_trajectory.py
        roslaunch object_tracking ar_track_alvar.launch
        roslaunch object_tracking darknet_ros.launch
        rosrun object_tracking detect_trashcan.py
        rosrun object_tracking find_trashcan.py
        rosrun object_tracking find_trash.py
        roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 

# Semi-autonomous operation
In this mode, you need to tele-op the robot in the beginning for it to see the water bottle and trashcan. Once it see those items, it will remember their locations even if you drive it to another location in the building. It can then be switched to fully autonomous mode: automatically approach water bottle --> pickup bottle --> approach trashcan --> dropoff bottle.



# Fully autonomous operation



# Room for improvement

## Make .rosinstall file to install all necessary packages at once

## Make launch file to launch all neccessary packages at once









