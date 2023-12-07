# ROS_Voicer Packages

This will describe how to install and use all the packages present within this ROS_Voicer_Packages folder. ROS Noetic on Ubuntu 20.04 was used on both the robot's internal computer and the PC in the lab, so make sure it is installed.

## utari10_jackal

A custom repository for the UTARI10_Jackal complete with ZED Camera and Kinova Jaco-2 manipulator arm. Originally provided by Clearpath Robotics when the UTARI10_Jackal was commissioned -- Now it contains additional custom hardware launch files to launch the arm and car sections as one unified robot description. Also contains the ROS_Voicer voice_command package within it.

### Adding accessories to main Jackal hardware description
The custom urdf file that includes the arm and ZED Camera accessories must be added to the Jackal's .urdf file before running this package.
In your workspace, navigate to the file jackal.urdf.xacro in your catkin workspace; it should be in the path:

```
~/catkin_ws/jackal/jackal_description/urdf/
```
Open this file with your favorite text editor, and at the end of the file, add the following line:

```
<xacro:include filename="$(find utari10_jackal)/urdf/jackal_description.urdf.xacro" />
```

Also create a fixed joint to attach the accessories to the Jackal. Must be in this format:
```
<link name="link_name"></link>
  <joint name="joint_name" type="fixed">
    <parent link="link_on_jackal" />
    <child link="j2n6s300_link_base" />
    <origin xyz="${mount_spacing} 0 0" />
  </joint>
```

### Dependencies

1. [jackal](https://github.com/jackal/jackal)
2. [jackal_robot](https://github.com/jackal/jackal_robot)
3. [kinova_ros](https://github.com/Kinovarobotics/kinova-ros)
4. [moveit](https://github.com/ros-planning/moveit)
5. [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper)    

### Running the Packages
  
To run it, make sure your workspace is sourced, and run:

```
roslaunch utari10_jackal utari10_jackal.launch
```


## utari10_jackal_moveit_config

MoveIt! Configuration created for the UTARI10_Jackal arm using the moveit_setup_assistant package. Allows for control of the chassis with interactive markers and motion planning for the arm. 

### Dependencies
**utari10_jackal** and all of it's dependencies

To run the MoveIt! visualization created by **utari10_jackal_moveit_config**, run this command:  
```
roslaunch utari10_jackal utari10_moveit.launch
```

## ROS_Voicer

A package that allows for voice-commanded control of the UTARI10_Jackal platform in conjunction with the aforementioned packages. Written in Python 3.8.10. 

### ROS Dependencies
**utari10_jackal** and all of its dependencies

### Python Packages needed

pyzed - ZED Camera Python API - comes bundled with ZED SDK.    
[ZED Python API Install Instructions](https://docs.stereolabs.com/app-development/python/install/)

#### From ROS
1. actionlib
2. geometry_msgs
3. kinova_msgs
4. tf
#### From Python Standard Library
1. math
2. threading
3. time
#### From PIP
1. OpenCV -[cv2](https://pypi.org/project/opencv-python/)
2. Playsound -[playsound](https://pypi.org/project/playsound/)
3. Termcolor - [termcolor](https://pypi.org/project/termcolor/)
4. YOLOv5 - [yolov5](https://github.com/ultralytics/yolov5)

    
### Running the Script
  
To run it, navigate to the ROS_Voicer_Python_Files directory, and run:

```
python3 ros_voicer.py
```



    
