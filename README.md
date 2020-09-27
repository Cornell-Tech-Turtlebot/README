# 1. Introduction

As of September 2020, our Trash-picking Turtlebot can autonomously navigate around an unknown building, draw a map, detect & pickup water bottle then drop into trashcan. See the video here: 

https://youtu.be/wEwvhGS8GY8

https://drive.google.com/drive/folders/1VAmSW8Z5EUJZb4Y-RXb9zj3yCQSbMGp9

![Trash-picking Turltlebot](/images/trash-bot.png)

It requires these packages:
- Object tracking (https://github.com/Cornell-Tech-Turtlebot/object_tracking): Detect different objects including water bottle & trashcan. Localize positions of those objects on the map. Drive the robot toward those objects.
- darknet_ros (https://github.com/leggedrobotics/darknet_ros): YOLO on ROS. Detect different objects including the water bottle.
- ar_track_alvar (http://wiki.ros.org/ar_track_alvar): Detect AR tag that is sticked to the trashcan.
- Bottle manipulator (https://github.com/Cornell-Tech-Turtlebot/bottle_manipulator): Control the manipulator to pickup & drop off the water bottle.
- explore_lite (http://wiki.ros.org/explore_lite): Autonomously explore & map an unknown building.  
- Patrol (https://github.com/Cornell-Tech-Turtlebot/patrol): Randomly patrol around the mapped building.
- Orchestrator (https://github.com/Cornell-Tech-Turtlebot/orchestrator): Orchestrate other packages. For example, when `object_tracking` successfully drives the robot to approach the bottle, `orcheschator` will tell `bottle_manipulator` to start picking the bottle up.

Follow this guide step-by-step to replicate what you see in the videos above.

# 2. Setup

## 2.1 Your laptop:
Follow this to setup ROS on your laptop:
https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup
https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation

Follow this to get your laptop into the same VPN as the robot & server: 
https://github.com/travers-rhodes/gazebo_tensorflow_turtlebot_docker#connection-instructions




## 2.2 Robot (optional):
The robots are already fully setup. The passwords to SSH into the robots:

- Waffle Pi: `admin`
- Waffle Pi with Open Manipulator: `turtlebot`

In case you need to setup the robots again, follow this tutorial: 

- https://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup
- https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation
- Calibrate the camera (important for tag detection to work): 
https://github.com/UbiquityRobotics/raspicam_node
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

If new servos are being used for the OpenManipulator, follow these instructions to program the Dynamixel servos:
- https://github.com/Cornell-Tech-Turtlebot/DINAMIXEL_OPEN_MANIPULATOR_CONFIGURATION

## 2.3 Server (optional):
The workspace `/catkin_ws` on server is fully setup, so it's best to just use it. If you decide to create a new workspace or virtual machine, remember to backup the previous `/catkin_ws` folder. There might be certain small configs we made to get things working, and we might miss it in this documentation. It's pretty hard to config things to make the server & robot work smoothly!

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

- Build MoveIt Package from Source:
               
        git clone https://github.com/ros-planning/moveit.git
                
  Find the move_group.py file in the repo (catkin_ws/src/moveit/moveit_commander/src/moveit_commander) and change the `wait_for_servers` parameter under `__init__` to `=50.0` (or higher value, as required).
        
  Remove the original moveit package installed with ROS Kinetic located in `/opt/ros/kinetic/lib/python2.7/dist-packages/`

 - Build the workspace:
 
        cd /catkin_ws
        catkin build
        source devel/setup.bash

   If `catkin_build` doesn't work, try `catkin_make_isolated` then `source devel_isolated/setup.bash`
   
 - Add this at the end of your `~/.bashrc` file:
 
        export TURTLEBOT3_MODEL=waffle_pi
        export ROS_MASTER_URI=http://10.8.0.1:11311
        export ROS_IP=10.8.0.1
        export C_INCLUDE_PATH=/usr/include/python2.7
        
   Then `source ~/.bashrc`
   
## 2.4 Physical equipments:
You need 1 water bottle & 1 trashcan. Print this AR tag and stick it to the trashcan:
https://github.com/Cornell-Tech-Turtlebot/object_tracking/blob/master/markers/MarkerData_0.png

![Trashcan](/images/trash-can.png)
![Bottle](/images/bottle.png)

Measure the side of the black square of the AR tag, in centimeter. Then run this on the **Server**:

        cd /catkin_ws/src/object_tracking/launch/ar_track_alvar.launch
        
Change the number in the second line to the side of the black square you've just measured. It should look something like this:

        <arg name="marker_size" default="20" />

# 3. Launch basic packages
## 3.1 On Robot:
Run each command below in a separate Terminal window:

        roslaunch turtlebot3_bringup turtlebot3_robot.launch
        roslaunch turtlebot3_bringup turtlebot3_rpicamera_2.launch    

## 3.2 On Server:
Run each command below in a separate Terminal window:
        
        rosrun image_transport republish compressed in:=/raspicam_node/image out:=/raspicam_node/image_raw
        rosrun tf static_transform_publisher 0 0 0 0 0 0 camera_rgb_optical_frame raspicam 100
        roslaunch object_tracking turtlebot3_slam.launch
        roslaunch turtlebot3_navigation move_base.launch
        roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
        vglrun rviz

When Rviz opens, click `File` --> `Open Config` --> navigate to `/catkin_ws/src/object_tracking/rviz/` --> select `object_tracking.rviz`. You will see Rviz interface displaying all neccessary information.

![Rviz](/images/rviz.png)

You can control the robot using teleop, in the Terminal window that you run `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`.

Better yet, you can use Rviz to set a location in the map for the robot to move there. Follow this instruction:
http://wiki.ros.org/rviz/UserGuide#A2D_Nav_Goal_.28Keyboard_shortcut:_g.29

# 4. Autonomous mapping of an unknown building
On **Server**, run this command in a new Terminal window:

        roslaunch explore_lite explore.launch

The robot will automatically explore & map unknown areas of building,  until there is no unknown area left.

# 5. Semi-autonomous trash picking
In this mode, you need to tele-op the robot in the beginning for it to see the water bottle and trashcan. Once the robot see those items, it will remember their locations, even if you drive the robot to look away. It can then automatically return to find & approach water bottle --> pickup bottle --> find & approach trashcan --> dropoff bottle. This was what we did in the video demos you see above.

Follow these steps:

## 5.1 Launch packages:
On **Server**, run each command in a separate Terminal window:   

        roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
        roslaunch turtlebot3_manipulation_moveit_config move_group.launch
        rosrun bottle_manipulation execute_trajectory.py
        roslaunch object_tracking ar_track_alvar.launch
        roslaunch object_tracking darknet_ros.launch
        rosrun object_tracking detect_trashcan.py
        rosrun object_tracking find_trashcan.py
        rosrun object_tracking find_trash.py

## 5.2 Teleop the robot: 
Drive the robot so that it sees the water bottle and trashcan (with the AR tag sticked on it). Once the robot see those items, it will remember their locations.

## 5.3 Send command to the robot: 
By publishing message to `/state` topic. You can do this in Terminal. Open a new Terminal window, then run these commands: 

Ask the robot to approach the bottle:

        rostopic pub /state std_msgs/String approach_trash
        
Once the robot approached the bottle, press `Ctr + C` of `Command + C` to terminate. Then run another command to ask the robot to pickup the bottle:

        rostopic pub /state std_msgs/String pickup_trash
        
Once the robot picked up the bottle, press `Ctr + C` of `Command + C` to terminate. Then run another command to ask the robot to approach the trashcan:

        rostopic pub /state std_msgs/String approach_trashcan
        
Once the robot approached the trashcan, press `Ctr + C` of `Command + C` to terminate. Then run another command to ask the robot to drop the bottle into the trashcan:

        rostopic pub /state std_msgs/String drop_trash

# 6. Fully autonomous trash picking
For the robot to be fully autonomous, we need to run the `orchestrator` package. This will orchestrate starting / stopping other packages & actions of the robot. We have written the code but didn't have enough time to test it before project ends. So you have the opportunity to finish it :)

The `orchestrator` package communicates with other packages via `/state` topic. All packages subscribe & publish to that `/state` topic. It works like this:

- When `explore_lite` finishes mapping the building, it will send `explore_done` message to `/state` topic
- When `orchestrator.py` receives that message, it will send `patrol` message to `/state` topic
- When `patrol` package receives that message, it will make the robot patrol (randomly go around the building)
- During patrolling, `detect_trashcan.py` and `detect_trash.py` will detect trashcan & bottle. Once detected, they will send `trashcan_detected` and `trash_detected` messages to `/state` topic
- When `orchestrator.py` receives those messages, it will send `approach_trash` message to `/state` topic
- When `find_trash.py` receives that message, it will drive the robot to approach the bottle, then send `approach_trash_done` message
- When `orchestrator.py` receives that message, it will send `pickup_trash` message to `/state` topic
- When `execute_trajectory.py` receives that message, it will make the robot pickup the bottle, then send `pickup_trash_done` message
- When `orchestrator.py` receives that message, it will send `approach_trashcan` message to `/state` topic
- When `find_trashcan.py` receives that message, it will drive the robot to approach the trashcan, then send `approach_trashcan_done` message
- When `orchestrator.py` receives that message, it will send `drop_trash` message to `/state` topic
- When `execute_trajectory.py` receives that message, it will make the robot drop the bottle into the trashcan

Take a look at this code file & you will understand: https://github.com/Cornell-Tech-Turtlebot/orchestrator/blob/master/src/orchestrator.py

To launch `orchestrator` package & `patrol` package, run each of this in a new Terminal window:

        roslaunch patrol random_nav.launch
        rosrun orchestrator orchestrator.py


# 7. Room for improvement

## 7.1 Make .rosinstall file to install all necessary packages at once
As you can see in Section 2, we need to install a bunch of packages. It's better to group them into a rosinstall file, so we can just run 1 file to install all of them.

## 7.2 Make launch file to launch all neccessary packages at once
As you can see, we need to launch a bunch of packages in seperate Terminal window. It's better to group them into 1 launch file and launch all at once.

## 7.3 Finish orchestrator package
As mentioned in section 6, we have written the code but haven't tested. We entrusted it to you to finish it :)

## 7.4 Tag detection is not accurate sometimes
Currently, we are using AR tag detection engine called `ar_track_avlar` (http://wiki.ros.org/ar_track_alvar). It works pretty well most of the times in a well lit room, but sometimes it mis-detects the position of the tag & results in the robot driving to the wrong location.

We also experimented with another tag detection engine - April tag (http://wiki.ros.org/apriltag_ros), which seems to be more accurate, but at the cost of slower detection speed. It runs well on local laptop, but on server it's pretty laggy, making real-time detection & driving difficult. We could probably improve the networking settings to reduce the latency, but haven't figured it out.

To try April tag detection, do this:

- Install apriltag_ros on server and your laptop, following this instruction: https://github.com/AprilRobotics/apriltag_ros#quickstart

- Comment line 24 & uncomment line 22 in this file: https://github.com/Cornell-Tech-Turtlebot/object_tracking/blob/master/scripts/detect_trashcan.py

- Print tag #10 from this file, then stick it to the trashcan: https://github.com/Cornell-Tech-Turtlebot/object_tracking/blob/master/markers/apriltags1-20.pdf

- Measure the size of 1 side of the black square on your printed tag, in meter. Then change the number in line 22 of this file to that number you've just measured: https://github.com/Cornell-Tech-Turtlebot/object_tracking/blob/master/markers/tags.yaml

- Copy these 2 files to folder `/catkin_ws/src/apriltag_ros/config/`:
        
        https://github.com/Cornell-Tech-Turtlebot/object_tracking/blob/master/markers/tags.yaml
        https://github.com/Cornell-Tech-Turtlebot/object_tracking/blob/master/markers/settings.yaml

- In Section 5.1, replace this command:

        roslaunch object_tracking ar_track_alvar.launch

     with this command:

        roslaunch object_tracking apriltag_ros.launch

- In RViz, click `Add` button --> `By topic` --> `tag_detections_image` --> `Image`. Rviz will then display the robot's camera, which will show the the detected April tag if the tag is in front of the camera.

![Rviz April tag](/images/rviz-april.png)

- Now you can continue to run the rest of the robot like usual. The robot will now detect April tag instead of AR tag to find the trashcan.








