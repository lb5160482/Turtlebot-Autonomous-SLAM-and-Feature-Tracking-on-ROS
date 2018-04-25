# Turtlebot Autonomous SLAM and Feature Tracking on ROS
<p align="center">
<img src="https://insights.ubuntu.com/wp-content/uploads/e203/ROS.png"   width="200px" height="100px" align="center"/>
</p>
The Main idea of this project is to use one Kinect-TurtleBot robot to autonomously explore the lab room and generate a 2D floor map, then it will delivery objects from one person to the other. After generating the map autonomously, the robot will navigate itself to one of the teammates where there is an AR tag to guide the robot. The person waits at the first position and loads the delivery object on the turtlebot. After that, the robot will navigate itself to a rough area in the other half of the lab room using the map it previously generated.Then the robot will move towards another person (this feature also can be triggered by the voice control), where there is a purple color tag to let the robot track. Another person waiting there will pick up the delivery object from the turtlebot and the object delivery part completes. We add a joystick to select the working mode of the robot and we can control the whole process using a joystick.The instructions on the joystick are shown below: 
<p align="center">
<img src="https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/imgs/joystick.png" width="530px" height="270px"/>
</p>

## 0. Index

1. [Nodes Created](#1-nodes-created)
2. [Launch Files](#2-launch-files)
3. [Existing Software Packages](#3-existing-software-packages)
4. [Sample Data](#4-sample-data)
5. [Video](https://www.youtube.com/watch?v=TV-s2yyTPh0 )

## 1. Nodes Created

### [ARtracker.cpp](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/turtlebot_project/src/ARtracker.cpp)
This node subscribes from “/ar_pose_marker” topic and sends message to "/mobile_base/commands/velocity" topic. According to the relative position of the AR tag, it does the path planning and sends the Turtlebot velocity message to the topic.   
### [colorTracker.cpp](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/turtlebot_project/src/colorTracker.cpp)
This node subscribes from “/blobs_3d” topic and sends message to "/mobile_base/commands/velocity". According to the relative position of the color blob, it does the path planning and sends the Turtlebot velocity message to the topic.  
### [mapsaver.cpp](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/turtlebot_project/src/mapsaver.cpp)  
This node subscribes from “/joy” topic and runs the bash command to save the map when the X button on the joystick is pressed.  
### [startArTracking.cpp](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/turtlebot_project/src/startArTracking.cpp)
This node subscribes from “/joy” topic and runs the bash command to start tracking the AR tagwhen the START button on the joystick is pressed.  
### [startColorTracking.cpp](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/turtlebot_project/src/startColorTracking.cpp)
This node subscribes from “/joy” topic and runs the bash command to start tracking the color when the LB button on the joystick is pressed.
### [viewNavigation.cpp](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/turtlebot_project/src/viewNavigation.cpp)
This node subscribes from “/joy” topic and runs the bash command to view Turtlebor navigation in Rviz when the BACK button on the joystick is pressed.  
### [startVoice.cpp](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/turtlebot_project/src/startVoice.cpp)
This node subscribes from “/joy” topic and runs the bash command to view start the voice control nodes when the right axle on the joystick is pressed.  
### [Selector.py](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/turtlebot_project/src/selector.py)
This node subscribes from “/joy” topic and runs corresponding bash commands to: 
* 1. Stop AR tag tracking when A button on the joystick is pressed.  
* 2. Stop color tracking when RB button on the joystick is pressed. 
* 3. Kill all the nodes to top the system by pressing the mid button on the joystick when emergency happens  

## 2. Launch Files

### [Startup.launch](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/turtlebot_project/launch/startup.launch)
This launch file firstly launches minimal.launch in turtlebot_bringup package to start the necessary nodes for running the turtlebot. Then it launches 3dsensor.launch in turtlebot_bringup package to get depth image from the kinect. This launch file is run on the turtlebot.
### [AutoMapping.launch](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/turtlebot_project/launch/AutoMapping.launch)
This launch file start autonomous mapping by using the operator node in nav2d_operator package to do autonomous obstacle avoidance and navigator node in nav2d_navigator package to do path planning cooperating with launching the gmapping.launch to do the SLAM. This launch file is run on the workstation.  
### [local.launch](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/turtlebot_project/launch/local.launch)
This launch file launches a bunch of nodes we wrote to do the specific tasks. This launch file is run on the workstation.  
### [trackPass.launch](https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/turtlebot_project/launch/trackPass.launch)
This launch file loads the map the robot generated, launches the nodes in turtlebot_navigation package, the ar_track_alvar node in ar_track_alvar package to recognize the AR tag and nodes in cmvision_3d package to recognize the specific color we set. This launch file is run on Turtlebot.

## 3. Existing Software Packages

### [Cmvision package](http://wiki.ros.org/cmvision)  
This package was used for fast color blob detection and usually combined with Cmvision_3d Packages for the 3D color tracking.  
### [Joy package](http://wiki.ros.org/joy)
Joy Package publishes topic(/joy) depend on the inputs of the controller and it includes the state of each one of the joystick's buttons and axes.
### [Pocketsohinx Package](http://wiki.ros.org/pocketsphinx)
This package includes a well-developed voice dictionary, a voice recognizer, and corresponding movements using python based interface.
### [AR_Track_Alvar Package](http://wiki.ros.org/ar_track_alvar)
This package is a ROS wrapper for Alvar, an AR tag tracking library. The published topic, ar_pose_marker, includes a list of the poses of the observed AR tag, with respect to the output frame.
### [Cmvision_3d Packages](http://wiki.ros.org/cmvision_3d)
Cmvision_3d uses the topic produced by cmvision to publish the position of each color blob relative to its camera frame, and the frames in the tf stack for each color.
### [Navigation_2D Package](http://wiki.ros.org/nav2d)
This package includes the obstacle avoidance, a simple path planner, and a graph based SLAM (Simultaneous Localization and Mapping) node that allows robots generate a map of a planar environment.
### [Gmapping](http://wiki.ros.org/gmapping)
The gmapping package provides a ros node (slam_gmapping), which has a laser-based SLAM feature.

## 4. Sample Data
<p align="center">
  <td><img src="https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/imgs/map.png" width="550px" height="300px"/></td>
</p>
<p align="center">Map Generated and Cost Map During Mapping</p>
<br>
<p align="center">
  <td><img src="https://github.com/lb5160482/Turtlebot-Autonomous-SLAM-and-Feature-Tracking-on-ROS/blob/master/imgs/tag%26color.png" width="550px" height="225px"/></td>
</p>
<p align="center">AR Tag Recognition and Color Recognition</p>

## 5. Video
https://youtu.be/JY9hPmRFG_k
