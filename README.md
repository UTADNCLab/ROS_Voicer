# ROS_Voicer
**Robotic Arm Control with Human Voice**

This is a project for The University of Texas at Arlington's Dynamic Networks and Control Laboratory. It's purpose is to command a robot arm to complete various tasks with simple speech commands. This is intended to be a comprehensive tutorial so that the project is completely replicable. This project existed in two phases. First I used a TurtleBot2i's PhantomX Pincher Arm, and then I moved on to applying the same principles to the Jaco2 Arm by Kinova Robotics.

## Required Software
1. **Operating System:** Ubuntu 20.04 Focal Fossa
2. **Robot Operating System (ROS) Version:** ROS Noetic Ninjemys
3. **Script Programming Language:** Python 3.8.10
4. **Code Editor:** Visual Studio Code

# TurtleBot Voice Commander
## Required Hardware
1. **Robot**: TurtleBot 2i
     - **Robot Arm** : Interbotix PhantomX Pincher Arm Mk3 (Only Shipped with TurtleBot 2i, not sold separately)
     - **Microcontroller**: ArbotiX-M Microcontroller (Can be bought standalone, included with TurtleBot 2i)
2. **USB-to-Serial-Connector**: FTDI TTL-232R-3v3 USB to Serial Converter
3. **A Linux Compatible Computer with USB Ports. My Choice:** Samsung Galaxy Book Flex Alpha (2019) 
4. **A Microphone. My Choice:** Built-In Mic on Laptop

The turtlebot should have come with A/C aapters to both charge the base's battery and to provide power to the ArbotiX-M Board. If the board and arm are being used standalone however, you would need to use a **12V, 5A A/C adapter** to power the board and Arm.

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
     - termcolor
     - numpy
# Jaco Voice Commander
