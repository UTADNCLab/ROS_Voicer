#! /usr/bin/env python
#!/usr/bin/python -tt
import os, struct, wave, pvporcupine, pvrhino, sys, roslaunch, copy, rospy, subprocess, faulthandler, threading
from datetime import datetime
import time
from pvrecorder import PvRecorder
from std_msgs.msg import Float64
import numpy as np
from playsound import playsound as ps
import tkinter as tk
from tkinter import *
from termcolor import colored

#function.py - Main Script for Voice Control

######################################################################################################################################################
# 1. ALL FUNCTIONS USED IN THE PROGRAM DECLARED HERE AT THE BEGINNING
######################################################################################################################################################

# jointrad function - Updates Joint States when called
def jointrad(state):
    pub3.publish(state[2]);pub2.publish(state[1]);pub1.publish(state[0]);
    pub4.publish(state[3]);pub5.publish(state[4]);
    rate.sleep()
    rospy.loginfo("Joint Angle is: [%1.2f %1.2f %1.2f %1.2f %1.2f] ",
                 state[0],state[1],state[2],state[3],state[4])

def splash_gui():
    #GUI - Splash Screen seen on startup

    # Create a new Toplevel window for the splash screen
    splash = tk.Toplevel()
    splash.title("My Application")
    splash.geometry("900x900")

    # Load the splash screen image
    image = tk.PhotoImage(file="Images/armpic.gif")

    # Create a label with the image and pack it into the Toplevel window
    labeling = tk.Label(splash, image=image)
    labeling.pack()
    
    # Schedule the splash screen to close after 3 seconds
    splash.after(5000, splash.destroy)
    splash.mainloop()    

# Enables 'faulthandler' - dumps Python tracebacks explicitly, on a fault, after a timeout, or on a user signal. 
faulthandler.enable()

######################################################################################################################################################
# 2. LAUNCHING NECESSARY ROS NODES 
######################################################################################################################################################

# Command to start launch file with ros as a subprocess - make sure to replace the path with where your arm.launch file is.
command = 'gnome-terminal --tab -- bash -c "cd ~/catkin_ws/src/pincher_arm/pincher_arm_bringup/launch/ && ls -l &&roslaunch arm.launch;exec bash"'
# Command to kill all processes (used later but declared here)
killall = 'gnome-terminal --tab -- bash -c "killall -9 roslaunch && killall -9 rosmaster && killall -9 rosout"'

# Launches command declared earlier as a subprocess
launcher = subprocess.call(['/bin/bash', '-i', '-c', command],stdout=True,stderr=True, text=True)

# PicoPublisher - the ros publishing node - publishes all joint states
rospy.init_node('PicoPublisher', anonymous=True)
rate = rospy.Rate(1) # 1 hz update rate for state publishing
pub1 = rospy.Publisher('/arm_shoulder_pan_joint/command', Float64, queue_size=10) # Servo 1 - Shoulder Pan
pub2 = rospy.Publisher('/arm_shoulder_lift_joint/command', Float64, queue_size=10)# Servo 2 - Shoulder Lift
pub3 = rospy.Publisher('/arm_elbow_flex_joint/command', Float64, queue_size=10)   # Servo 3 - Elbow
pub4 = rospy.Publisher('/arm_wrist_flex_joint/command', Float64, queue_size=10)   # Servo 4 - Wrist
pub5 = rospy.Publisher('/gripper_joint/command', Float64, queue_size=10)          # Servo 5 - Gripper

######################################################################################################################################################
# 3. CREATING AND LAUNCHING PICOVOICE SPEECH ENGINES - PORCUPINE (WAKEWORD), RHINO(SPEECH-TO-INTENT) , & PVRECORDER ((SPEECH RECORDER AND PARSER)
######################################################################################################################################################

# declaring args for picovoice modules before using them
access_key = 'BKZ6QIyCZPIirHUNrqCmxR8mwVJHIPSGFv22P2/rFGpqJilSGLmf/g=='# Required Picovoice Access Key
keyword_paths = ['~/Desktop/Picovoice/Porcupine/Hey-Turtle-bot.ppn'] # Porcupine .ppn file path
context_path = "/home/carlos/Desktop/Picovoice/Rhino/Arm.rhn" # Rhino .rhn file path

# Creating Porcupine wake word engine and Rhino speech-to-intent engine instances
porcupine = pvporcupine.create(access_key=access_key,keyword_paths=keyword_paths) 
rhino = pvrhino.create(access_key=access_key,context_path=context_path)

# Printing Module Versions and Context information for clarity
print('Porcupine version: %s' % porcupine.version)
print('Rhino version: %s' % rhino.version)
print('Context info: %s' % rhino.context_info)

# Creating and starting Recorder Instance
recorder = PvRecorder(device_index=5, frame_length=porcupine.frame_length)

# Parses keywords from porcupine
keywords = list()
for x in keyword_paths:
    keyword_phrase_part = os.path.basename(x).replace('.ppn', '').split('_')
    if len(keyword_phrase_part) > 6:
        keywords.append(' '.join(keyword_phrase_part[0:-6]))
    else:
        keywords.append(keyword_phrase_part[0])

# Creates a wav file to store audio stream while in use
wav_file = wave.open('wave.wav', "w")
wav_file.setnchannels(1)
wav_file.setsampwidth(2)
wav_file.setframerate(16000)

#variable to start Splash Screen as a separate thread 
splash_thread = threading.Thread(target=splash_gui)

######################################################################################################################################################
# MAIN SECTION OF CODE
######################################################################################################################################################

if __name__ == '__main__':
    try:
        #Run Splash screen.
        splash_thread.start()
        #Startup Sound - anything with ps is just playing the .wav files in the Audio folder. Also made the terminal print out what turtlebot says.
        ps("Audio/o95.wav")
        
        print(colored('TURTLEBOT SAYS:','blue')+" Hello There!")
        ps('Audio/hello.wav')

        print(colored('TURTLEBOT SAYS:','blue')+" I am TurtleBot.")
        ps('Audio/turtlebot.wav')

        print(colored('TURTLEBOT SAYS:','blue')+"I'm Listening...")
        ps('Audio/listening.wav')

        # Allows us to see selected audio device - can be changed with args 
        print('Using device: %s' %recorder.selected_device)
        # Let client know script is currently listening
        print('Listening ... (press Ctrl+C to exit)')
    
        print("Moving to 'NEUTRAL'State...")
        state = np.array([-1.8,-1.4,-1,1.5,0])
        jointrad(state)

        recorder.start()
        while True:
            
            #make recorder parse speech from wave.wav
            pcm = recorder.read()
            # Both Rhino and Porcupine are made to process it.
            result = porcupine.process(pcm)
            is_finalized = rhino.process(pcm)

            # storing wav data in array
            if wav_file is not None:
                wav_file.writeframes(struct.pack("h" * len(pcm), *pcm))

            # if wakeword is detected, stop and move to neutral
            if result >= 0:
                recorder.stop()
                print('[%s] Detected %s' % (str(datetime.now()), keywords[result]))
                print("Moving to 'NEUTRAL'State...")
                state = np.array([-1.8,-1.4,-1,1.5,0])
                jointrad(state)
                print(colored('TURTLEBOT SAYS:','blue')+"I'm Listening...")
                ps('Audio/listening.wav')
                recorder.start()
                
            if is_finalized:
                inference = rhino.get_inference()
                if inference.is_understood:
                    print("  intent : '%s'" % inference.intent)
                    for slot, value in inference.slots.items():
                        print("  %s : '%s'" % (slot, value))
                        direction = value

                    #stop recorder to prevent overflow error    
                    recorder.stop()
                    
                    if inference.intent == 'move':

                        #move to Up state
                        if direction =='up':
                            state = np.array([-1.8,0,1.5,1.5,0])
                            print(colored('TURTLEBOT SAYS:','blue')+" Moving Up.")
                            ps('Audio/up.wav')
                            rospy.loginfo("Moving to 'UP' state...")
                            jointrad(state)
                            

                        #move to Down state
                        elif direction =='down':
                            state = np.array([-1.8,-1.7,1.5,1.5,0])
                            print(colored('TURTLEBOT SAYS:','blue')+" Moving Down.")
                            ps('Audio/down.wav')
                            rospy.loginfo("Moving to 'DOWN' state...")
                            jointrad(state)
                            
                        #move to Left state
                        elif direction =='left':
                            state = np.array([-1,-1.4,-1,1.5,0])
                            print(colored('TURTLEBOT SAYS:','blue')+" Moving Left.")
                            ps('Audio/left.wav')
                            rospy.loginfo("Moving to 'LEFT' state...")
                            jointrad(state)
                            
                        #move to Right state
                        elif direction =='right':
                            state = np.array([-2.6,-1.4,-1,1.5,0])
                            print(colored('TURTLEBOT SAYS:','blue')+" Moving Right.")
                            ps('Audio/right.wav')
                            rospy.loginfo("Moving to 'RIGHT' state...")
                            jointrad(state)
                            
                        else:
                            pass
                    #start recorder to continue listening.
                    recorder.start()
                    
                    # wave motion
                    if inference.intent == 'sayHi':
                        recorder.stop()
                        print(colored('TURTLEBOT SAYS:','blue')+" Hello There!")
                        ps('Audio/hello.wav')
                        rospy.loginfo("Waving...")
                        rospy.loginfo("Moving to 'LEFT' state...")
                        
                        state = np.array([-1,-1.4,-1,1.5,0])
                        jointrad(state)
                                    
                        state = np.array([-2.6,-1.4,-1,1.5,0])
                        rospy.loginfo("Moving to 'RIGHT' state...")
                        jointrad(state)
                        
                        state = np.array([-1,-1.4,-1,1.5,0])
                        rospy.loginfo("Moving to 'LEFT' state...")
                        jointrad(state)
                        
                        rospy.loginfo("Moving to 'RIGHT' state...")
                        state = np.array([-2.6,-1.4,-1,1.5,0])
                        jointrad(state)
                        
                        state = np.array([-1,-1.4,-1,1.5,0])
                        rospy.loginfo("Moving to 'LEFT' state...")
                        jointrad(state)
                        
                        rospy.loginfo("Moving to 'RIGHT' state...")
                        state = np.array([-2.6,-1.4,-1,1.5,0])
                        jointrad(state)
                        
                        state = np.array([-1,-1.4,-1,1.5,0])
                        rospy.loginfo("Moving to 'LEFT' state...")
                        jointrad(state)
                        
                        rospy.loginfo("Moving to 'RIGHT' state...")
                        state = np.array([-2.6,-1.4,-1,1.5,0])
                        jointrad(state)
                        ps('Audio/tada.wav')
                        recorder.start()
                        
                    if inference.intent =='pickup':
                        recorder.stop()
                        print("Picking up object...")
                        state = np.array([-1.7,-1.4,-1,1.5,0])
                        rospy.loginfo("Moving to 'LEFT' state...")
                        jointrad(state)
                        rate.sleep()

                        state = np.array([-1.7,-1.7,1.5,1.5,0])
                        rospy.loginfo("Moving to 'DOWN' state...")
                        jointrad(state)
                        rate.sleep()

                        print(colored('TURTLEBOT SAYS:','blue')+" Grabbing Object...")
                        ps('Audio/grabbing.wav')
                        state = np.array([-1.7,-1.7,1.5,1.5,-2.2])
                        rospy.loginfo("Moving to 'CLOSING' state...")
                        jointrad(state)
                        rate.sleep()
                        rate.sleep()
                        rate.sleep()

                        state = np.array([-1.7,0,1.5,1.5,-2.2])
                        rospy.loginfo("Moving to 'UP' state...")
                        jointrad(state)
                        
                        print("Placing down object...")

                        rospy.loginfo("Moving to 'RIGHT' state...")
                        state = np.array([-2.6,0,1.5,1.5,-2.2])
                        jointrad(state)

                        state = np.array([-2.6,-1.7,1.5,1.5,-2.2])
                        rospy.loginfo("Moving to 'DOWN' state...")
                        jointrad(state)
                        rate.sleep()
                        
                        print(colored('TURTLEBOT SAYS:','blue')+" Releasing Object...")
                        ps('Audio/releasing.wav')
                        rospy.loginfo("Moving to 'Opening' state...")
                        state = np.array([-2.6,-1.7,1.5,1.5,0])
                        jointrad(state)
                        rate.sleep()
                        rate.sleep()
                        rate.sleep()
                        

                        state = np.array([-1.8,-1.4,-1,1.5,0])
                        jointrad(state)
                        ps('Audio/tada.wav')
                        recorder.start()

                    if inference.intent == 'shutdown':
                        recorder.stop()
                        print(colored('TURTLEBOT SAYS:','blue')+"Shutting Down. Goodbye!")
                        ps('Audio/shutdown.wav')
                        print("Resetting to 'NEUTRAL'State...")
                        state = np.array([-1.8,-1.4,-1,1.5,0])
                        jointrad(state)
                        ps('Audio/tada.wav')
                        launcher = subprocess.call(['/bin/bash', '-i', '-c', killall],stdout=True,stderr=True, text=True)
                        exit(0)

    except KeyboardInterrupt:
        print('Stopping ...')
    finally:
        recorder.delete()
        porcupine.delete()
        rhino.delete()
        if wav_file is not None:
            wav_file.close()
