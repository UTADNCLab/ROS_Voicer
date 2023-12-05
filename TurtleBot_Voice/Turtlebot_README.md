# TurtleBot Voice Commander

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
