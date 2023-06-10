# ROS_Voicer
**Robotic Arm Control with Human Voice**

This is a project for The University of Texas at Arlington's Dynamic Networks and Control Laboratory. It's purpose is to command a robot arm to complete various tasks with simple speech commands. This is intended to be a comprehensive tutorial so that the project is completely replicable. This project existed in two phases. First I used a TurtleBot2i's PhantomX Pincher Arm, and then I moved on to applying the same principles to the Jaco2 Arm by Kinova Robotics.

**For both programs, my computer of choice was my personal laptop, a Samsung Galaxy Book Flex Alpha (2019), using the built in microphone. Not sure of the processing and memory requirements but this model had an Intel Core i7 (10th Gen) processor, 12 GB of RAM, and a 512 GB SSD, 256 of which was partitioned for the Linux Distro. You may or may not need a machine with comparable specs to be able to achieve this comfortably.**

## Required Software - For Both Robots
1. **Operating System:** Ubuntu 20.04 Focal Fossa
2. **Robot Operating System (ROS) Version:** ROS Noetic Ninjemys
3. **Script Programming Language:** Python 3.8.10
4. **Code Editor:** Visual Studio Code

**Installing Ubuntu:** [Ubuntu 20.04 Focal Fossa Installation and Setup](https://github.com/UTADNCLab/ROS_Voicer/wiki/Ubuntu-20.04-Focal-Fossa-Installation-and-Setup)  
**Installing ROS:** [ROS Noetic Installation and Setup](https://github.com/UTADNCLab/ROS_Voicer/wiki/ROS-Noetic-Installation-and-Setup)

Once Ubuntu and ROS are set up, you can install other programs as necessary.

To install Python 3, run:

    sudo apt update
    sudo apt install python3.8 -y

Visual Studio Code is available in the default app marketplace for Ubuntu, the Snap Store, download it from there.

Once the packages are installed and the workspace is built, make sure you source ROS and your catkin workspace:

    source /opt/ros/noetic/setup.bash 
    source ~/catkin_ws/devel/setup.bash 
    
You can also automatically source them in every shell by running these commands:
    
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    
Auto sourcing ROS may be beneficial but doing the same with the catkin workspace but may not be preferable if you have different workspaces.

Now we can move on to using the voice command programs.

# TurtleBot Voice Commander

This section will cover all that is required to run the TurtleBot Voice Commander program. It will also cover how to use the program. More in-depth tutorials will be available in the wiki, and will be linked accordingly.

## Required Hardware
1. **Robot**: TurtleBot 2i
     - **Robot Arm** : Interbotix PhantomX Pincher Arm Mk3 (Only Shipped with TurtleBot 2i, not sold separately)
     - **Microcontroller**: ArbotiX-M Microcontroller (Can be bought standalone, included with TurtleBot 2i)
2. **USB-to-Serial-Connector**: FTDI TTL-232R-3v3 USB to Serial Converter

The turtlebot should have come with A/C adapters to both charge the base's battery and to provide power to the ArbotiX-M Board. If the board and arm are being used standalone however, you would need to use a **12V, 5A A/C adapter** to power the board and Arm.

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
## Using the TurtleBot Voice Commander

The main python script used is called function.py. Before using it, make sure the ArbotiX-M Microcontroller and PhantomX Pincher Arm are set up properly.

[ArbotiX-M Robocontroller Setup](https://github.com/UTADNCLab/ROS_Voicer/wiki/Arbotix-M-Robocontroller-Setup)  
[Pincher Arm Mk3 Setup](https://github.com/UTADNCLab/ROS_Voicer/wiki/Pincher-Arm-Mk3-Setup)

Once they are set up, ensure the ArbotiX-M is properly connected to the computer. Then navigate to function.py's folder and launch it. I kept the folder "TurtleBot_Voice" on my desktop but of course replace the path to function.py with your own.
    
    cd ~/Desktop/TurtleBot_Voice
    python3 function.py

After the Splash Screen, TurtleBot should greet you, reset to its home position, and tell you it is listening for commands.

###Possible Commands:
1. "Hey TurtleBot!" - will reset TurtleBot to home position and let you know he is awaiting command. It's a way to call attention to her.
2. " Move/Go" - will tell TurtleBot to move in a direction. Possible Directions are:
    - "Up"
    - "Down"
    - "Left"
    - "Right"
3. "Say Hello/Hi" - will cause TurtleBot to greet you and wave her hand.
4.  "Pick (it/that thing/the object) up [please]" - Turtlebot will go to a specific location, grab an object in that preset location, and place it in another preset location. 

If you are interested in how the script works, I have commented it thoroughly.  
[function.py](https://github.com/UTADNCLab/ROS_Voicer/blob/main/TurtleBot_Voice/function.py)  

# Jaco Voice Commander

This section will cover all that is required to run the Jaco Voice Commander program. It will also cover how to use the program. Just like for the Turtlebot, more in-depth tutorials will be available in the wiki, and will be linked accordingly.

## Required Hardware
1. **Robot**: Clearpath Robotics Jackal UGV 
     - **Robot Arm** : Kinova Robotics Jaco2 6 DOF Arm (Can be included as attachment to Jackal UGV, also sold separately)
     - **Microcontroller**: Microcontroller is included within the arm itself.
2. **USB-to-Serial-Connector**: USB 2.0 Type A Male to Type B Male Cable

The Jaco is powered directly from the Jackal UGV. The Jackal has its own battery pack custom made for the Jackal that can be charged while within the UGV or externally. This provides power to the arm and the rest of the UGV if you wish to use it.

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
## Using Jaco Voice Commander

Make sure you set up the Arm according to this page: [Setting Up the Kinova Jaco Arm](https://github.com/UTADNCLab/ROS_Voicer/wiki/Setting-Up-the-Kinova-Jaco-Arm)  

Of course, confirm the arm is plugged into the computer via USB and turned on.
Like for the TurtleBot Program, make sure your ROS distro and caktin workspace are properly sourced. 

I have not implemented the launch files necessary to run the script into it yet, so first you must launch both the arm's launch file and the MoveIt! launch file in two separate shells:

    roslaunch kinova_bringup kinova_robot.launch
    roslaunch j2n6s300_moveit_config j2n6s300_demo.launch
    
Once that is done you should be able to launch the main program file:

    python3 jacogui.py
    
This will launch a splash screen, and then and a GUI with Jaco's speech and status printed on the left, and joint states printed on the right. Jaco should introduce himself and then assure you he is listening to your voice.

### Possible Commands:
1. "Hey Jaco!" - will reset Jaco to home position and let you know he is awaiting command. It's a way to call attention to him.
2. " Move/Go" - will tell Jaco to move in a direction. Possible Directions are:
    - "Up"
    - "Down"
    - "Left"
    - "Right"
### MORE TO COME SOON!

If you are interested in how the script works, I have commented it thoroughly as well. 
[jacogui.py](https://github.com/UTADNCLab/ROS_Voicer/blob/main/Jaco_Voice/jacogui.py)



