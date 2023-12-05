# Jaco Voice Commander
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

If you are interested in how the script works, I have commented it thoroughly as well. 
[jacogui.py](https://github.com/UTADNCLab/ROS_Voicer/blob/main/Jaco_Voice/jacogui.py)
