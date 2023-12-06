# ROS_Voicer Packages

This will describe how to install and use all the packages present within this ROS_Voicer_Packages folder. ROS Noetic on Ubuntu 20.04 was used on both the robot's internal computer and the PC in the lab, so make sure it is installed.

## utari10_jackal

A custom repository for the UTARI10_Jackal complete with ZED Camera and Kinova Jaco-2 manipulator arm. Originally provided by Clearpath Robotics when the UTARI10_Jackal was commissioned -- Now it contains additional custom hardware launch files to launch the arm and car sections as one unified robot description. Also contains the ROS_Voicer voice_command package within it.

**ROS Dependencies:**

1. [jackal](https://github.com/jackal/jackal)
2. [jackal_robot](https://github.com/jackal/jackal_robot)
3. [kinova_ros](https://github.com/Kinovarobotics/kinova-ros)
4. [moveit](https://github.com/ros-planning/moveit)
5. [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper)    

## utari10_jackal_moveit_config

MoveIt! Configuration created for the UTARI10_Jackal arm using the moveit_setup_assistant package. Allows for control of the chassis with interactive markers and motion planning for the arm. 

**ROS Dependencies:**
**utari10_jackal** and all of it's dependencies

## ROS_Voicer

A package that allows for voice-commanded control of the UTARI10_Jackal platform in conjunction with the aforementioned packages. Written in Python 3.8.10. 

**ROS Dependencies:**
**utari10_jackal** and all of its dependencies

**Python Packages needed:**
1. [actionlib](https://wiki.ros.org/actionlib)
2. [cv2](https://pypi.org/project/opencv-python/)
3. [geometry_msgs](https://index.ros.org/p/geometry_msgs/)
4. [kinova_msgs]()
5. [math]()
6. [playsound]()
7. [pyzed]()
8. [rospy]()
9. [std_msgs]()
10. [termcolor]()
11. [tf]()
12. [threading]()
13. [time]()
14. [yolov5]()

To run it, navigate to the ROS_Voicer_Python_Files directory, and run:

```
python3 ros_voicer.py
```



    
