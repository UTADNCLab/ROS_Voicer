# ROS_Voicer
**Robotic Arm Control with Human Voice**

This is a project for The University of Texas at Arlington's Dynamic Networks and Control Laboratory. It's purpose is to command a robot arm to complete various tasks with simple speech commands. This is intended to be a comprehensive tutorial so that the project is completely replicable. This project existed in two phases. First I used a TurtleBot2i's PhantomX Pincher Arm, and then I moved on to applying the same principles to the Jaco2 Arm by Kinova Robotics.

## Required Software - For Both Robots
1. **Operating System:** Ubuntu 20.04 Focal Fossa
2. **Robot Operating System (ROS) Version:** ROS Noetic Ninjemys
3. **Script Programming Language:** Python 3.8.10
4. **Code Editor:** Visual Studio Code

**For both programs, my computer of choice was my personal laptop, a Samsung Galaxy Book Flex Alpha (2019), using the built in microphone. Not sure of the processing and memory requirements but this model had an Intel Core i7 (10th Gen) processor, 12 GB of RAM, and a 512 GB SSD, 256 of which was partitioned for the Linux Distro. You may or may not need a machine with comparable specs to be able to achieve this comfortably.**

# TurtleBot Voice Commander

This section will cover all that is required to run the TurtleBot Voice Commander program. It will also cover how to use the program. More in-depth tutorials will be available in the wiki, and will be linked accordingly.

## Required Hardware
1. **Robot**: TurtleBot 2i
     - **Robot Arm** : Interbotix PhantomX Pincher Arm Mk3 (Only Shipped with TurtleBot 2i, not sold separately)
     - **Microcontroller**: ArbotiX-M Microcontroller (Can be bought standalone, included with TurtleBot 2i)
2. **USB-to-Serial-Connector**: FTDI TTL-232R-3v3 USB to Serial Converter

The turtlebot should have come with A/C aapters to both charge the base's battery and to provide power to the ArbotiX-M Board. If the board and arm are being used standalone however, you would need to use a **12V, 5A A/C adapter** to power the board and Arm.

## Required Software
### ROS Packages Needed
1. arbotix_ros
2. pincher_arm

### Python Modules Needed
1. **Built-In Python Modules:**
     - os
     - struct
     - wave
     - sys
     - subprocess
     - faulthandler
     - datetime - datetime
2. **Picovoice Modules:**
     - pvporcupine
     - pvrhino
     - pvrecorder
3. **ROS Modules:**
      - roslaunch
      - rospy
      - std_msgs.msg - Float64
4. **Other Modules:**
     - tkinter
     - playsound
     - termcolor -colored
     - numpy
# Jaco Voice Commander

This section will cover all that is required to run the Jaco Voice Commander program. It will also cover how to use the program. Just like for the Turtlebot, more in-depth tutorials will be available in the wiki, and will be linked accordingly.

## Required Hardware
1. **Robot**: Clearpath Robotics Jackal UGV 
     - **Robot Arm** : Kinova Robotics Jaco2 6 DOF Arm (Can be included as attachment to Jackal UGV, also sold separately)
     - **Microcontroller**: Microcontroller is included within the arm itself.
2. **USB-to-Serial-Connector**: USB 2.0 Type A Male to Type B Male Cable

## Required Software

** ROS Package Needed: kinova-ros and all its dependencies **
### Python Modules Needed

1. **Built-In Python Modules:**
     - os
     - struct
     - wave
     - subprocess
     - threading
     - faulthandler
     - datetime - datetime
     - queue
2. **Picovoice Modules:**
     - pvporcupine
     - pvrhino
     - pvrecorder - PvRecorder
3. **ROS Modules:**
      - roslaunch
      - rospy
      - moveit_commander
      - kinova_msgs.msg - JointAngles
4. **Other Modules:**
     - PySimpleGUI
     - playsound
     - termcolor - colored




