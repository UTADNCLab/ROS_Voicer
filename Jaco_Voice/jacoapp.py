#! /usr/bin/env python
#!/usr/bin/python -tt
import os, struct, wave, pvporcupine, pvrhino, rospy, subprocess, faulthandler, threading
from datetime import datetime
from pvrecorder import PvRecorder
from std_msgs.msg import Float64
import numpy as np
from playsound import playsound as ps
import tkinter as tk
from tkinter import *
from termcolor import colored
import moveit_commander

#JacoApp.py - script for voice control with no GUI

######################################################################################################################################################
# 1. ALL FUNCTIONS USED IN THE PROGRAM DECLARED HERE AT THE BEGINNING
######################################################################################################################################################

# jointrad function - Updates Joint States when called - similar to turtlebot arm but uses moveit_commander instead
def jointrad(armstate):
    armstate = armstate
    #handstate = handstate

    arm.set_named_target(armstate)
    plan = arm.go(wait=True)  

    #gripper.set_named_target(handstate)
    #plan2 = gripper.go(wait=True)  

######################################################################################################################################################
# 2. LAUNCHING NECESSARY ROS NODES 
######################################################################################################################################################

#ROS node that will publish states to MoveIt!
rospy.init_node('JacoVoicer', anonymous=False)

# Instantiating Arm Group
group_name = 'arm'
arm = moveit_commander.MoveGroupCommander(group_name)
arm.set_max_velocity_scaling_factor(1.0)

# Instantiating Hand Group
group2_name = 'gripper'
gripper = moveit_commander.MoveGroupCommander(group2_name)
gripper.set_max_velocity_scaling_factor(1.0)

######################################################################################################################################################
# 3. CREATING AND LAUNCHING PICOVOICE SPEECH ENGINES - PORCUPINE (WAKEWORD), RHINO(SPEECH-TO-INTENT) , & PVRECORDER ((SPEECH RECORDER AND PARSER)
######################################################################################################################################################

# declaring args for picovoice modules before using them
access_key = '2saeQ0pr4C5VaUWzNDyGAM3vUMli/zt6efotQubsLICInVRXZCRWOA=='# Required Picovoice Access Key
keyword_paths = ['~/Desktop/Picovoice/Porcupine/Hey-Jaco.ppn'] # Porcupine .ppn file path
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
wav_file = wave.open('jacoaudio.wav', "w")
wav_file.setnchannels(1)
wav_file.setsampwidth(2)
wav_file.setframerate(16000)

######################################################################################################################################################
# MAIN SECTION OF CODE
######################################################################################################################################################

# Enables 'faulthandler' - dumps Python tracebacks explicitly, on a fault, after a timeout, or on a user signal. 
faulthandler.enable()

if __name__ == '__main__':
    try:
        #Startup Sound - anything with ps is just playing the .wav files in the Audio folder. Also made the terminal print out what JACO says.
        ps("Audio/boot.wav")
        
        print(colored('JACO says:','blue')+" Hello There!")
        ps("Audio/Hello There 2.wav")
        print(colored('JACO says:','blue')+" I am Jaco, your robotic assistant.")
        ps("Audio/I am Jaco.wav")
        

        print("Moving to 'Home'State...")
        armstate = 'Home'
        jointrad(armstate)
        print(colored('JACO says:','blue')+"I'm Listening...")
        ps("Audio/Listening.wav")

        # Allows us to see selected audio device - can be changed with args 
        print('Using device: %s' %recorder.selected_device)
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
                print("Moving to 'Home' State...")
                armstate = 'Home'
                jointrad(armstate)
                print(colored('JACO says:','blue')+"At your Service!")
                ps("Audio/At your service 1.wav")
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
                            armstate = 'Vertical'
                            print(colored('JACO says:','blue')+" Moving Up.")
                            rospy.loginfo("Moving to 'Up' state...")
                            ps("Audio/Moving Up 1.wav")
                            jointrad(armstate)
                            
                        #move to Down state
                        elif direction =='down':
                            armstate = 'Down'
                            print(colored('JACO says:','blue')+" Moving Down.")
                            rospy.loginfo("Moving to 'Down' state...")
                            ps("Audio/Moving Down 1.wav")
                            jointrad(armstate)
                            
                        #move to Left state
                        elif direction =='left':
                            armstate = 'Left'
                            print(colored('JACO says:','blue')+" Moving Left.")
                            ps("Audio/Moving Left 1.wav")
                            rospy.loginfo("Moving to 'Left' state...")
                            jointrad(armstate)

                        #move to Right state
                        elif direction =='right':
                            armstate = 'Right'
                            print(colored('JACO says:','blue')+" Moving Right.")
                            ps("Audio/Moving Right 1.wav")
                            rospy.loginfo("Moving to 'Right' state...")
                            jointrad(armstate)
                            
                        else:
                            pass
                    #start recorder to continue listening.
                    recorder.start()

                    if inference.intent == 'shutdown':
                        recorder.stop()
                        print("Moving to 'Retract' State...")
                        armstate = 'Retract'
                        jointrad(armstate)
                        print(colored('JACO Says:','blue')+"Shutting Down. Goodbye!")
                        ps("Audio/Shutting Down 1.wav")
                        ps("Audio/Goodbye 2.wav")
                        ps("Audio/turnoff.wav")
                        exit(0)

    except KeyboardInterrupt:
        print('Stopping ...')
    finally:
        recorder.delete()
        porcupine.delete()
        rhino.delete()
        if wav_file is not None:
            wav_file.close()