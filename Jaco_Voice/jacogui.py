#!/usr/bin/env python
import os, struct, wave, faulthandler, threading, pvporcupine, pvrhino, rospy, queue
from datetime import datetime
from pvrecorder import PvRecorder
from playsound import playsound as ps
from termcolor import colored
import moveit_commander
from PIL import Image
import PySimpleGUI as sg
from kinova_msgs.msg import JointAngles

# JacoGUI3.py - Jaco Voice Commander with GUI Added 
######################################################################################################################################################
# 1. CREATING FUNCTIONS TO LAUNCH AS INDEPENDENT THREADS
######################################################################################################################################################

# Joint Angle Listener - Listens to ROS topic where joint states are published. Made to print to GUI for user's benefit
def joint_angle_listener():

    joint_topic = "/j2n6s300_driver/out/joint_angles"
    rate = rospy.Rate(0.5)  # Modifiable rate that can be used to throttle ROS events. 
                            # Used here to throttle reception of topic messages since they are constant
    
    # While loop to receive data from said topic
    while not rospy.is_shutdown():
        message = rospy.wait_for_message(joint_topic, JointAngles, timeout=None)
        # Split message into an array
        joint_angles = [message.joint1, message.joint2, message.joint3, message.joint4, message.joint5, message.joint6, message.joint7]
        # round the array's values to the nearest integer - my preference
        joint_angles = [round(angle) for angle in joint_angles]
        # Add joint message to queue
        joint_queue.put(joint_angles)
        #Set joint event as active
        joint_event.set()
        # Delay action by specified rate
        rate.sleep()

# GUI_function - The function that will take care of the operation of the splash screen and GUI

def GUI_function():
    # Joint angles to be published
    j1= joint_angles[0];j2= joint_angles[1];j3= joint_angles[2];j4= joint_angles[3]
    j5= joint_angles[4];j6= joint_angles[5];j7= joint_angles[6]

    timeout = 4000 # Splash screen timeout
    speak = "..." # Initial value of Jaco speech variable
    status = "..." # Initial value of Jaco status variable

# This is the Splash Screen's Layout. It has a picture of Jaco on the Clearpath UGV and the words "JACO Voice Commander"
    splash_layout = [
    [sg.Text('Jaco Voice\n Commander',
            text_color='black',
            background_color='white',
            justification ='center',
            font=('Tw Cen MT', 72)),
    sg.Image(filename= 'Images/jackal.png')]
    ]

# This is the Main GUI's Layout. It has 2 columns. On the left it will have a picture of Jaco, a text box with Jaco's speech, and another with Jaco's status.
# The right will have all the joint states and an exit button.
    main_layout = [
        [sg.Column([
            [sg.Image(filename='Images/jaco.png', background_color='white')],
            [sg.Text('Jaco says:', text_color='black', background_color='white', justification='center',
                      font=('Tw Cen MT', 20))],
            [sg.Text(speak, text_color='lightblue', background_color='black', justification='left', size=(40, 1),
                      font=('Tw Cen MT', 20), key='-SPEAK-')],
            [sg.Text('Status:', text_color='black', background_color='white', justification='center',
                      font=('Tw Cen MT', 20))],
            [sg.Text(status, text_color='yellow', background_color='black', justification='left',
                      size=(40, 1), font=('Tw Cen MT', 20), key='-STATUS-')]
        ], background_color='white'),
         [sg.Column(
             [sg.Text('Joint States', text_color='blue', background_color='white', justification='center',
                       size=(20, 1), font=('Tw Cen MT', 30))],
             [sg.Text('1- Shoulder Lift:', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
                       [sg.Text(j1, text_color='Green', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20),key='-JS1-')],
             [sg.Text('2 - Shoulder Pan', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
                       [sg.Text(j2, text_color='Green', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20),key='-JS2-')],
             [sg.Text('3 - Elbow Flex', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
                       [sg.Text(j3, text_color='Green', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20),key='-JS3-')],
             [sg.Text('4 -Wrist Pan', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
                       [sg.Text(j4, text_color='Green', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20),key='-JS4-')],
             [sg.Text('5 - Wrist Lift', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
                       [sg.Text(j5, text_color='Green', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20),key='-JS5-')],
             [sg.Text('6 - Wrist Rotate', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
                       [sg.Text(j6, text_color='Green', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20),key='-JS6-')],
             [sg.Text('7 - Hand Rotate', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
                       [sg.Text(j7, text_color='Green', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20),key='-JS7-')],
             [sg.Button('EXIT')],
             background_color='white')]  
         ],
         [sg.Column([
             [sg.Text('Joint States', text_color='blue', background_color='white', justification='center',
                       size=(20, 1), font=('Tw Cen MT', 30))],
             [sg.Text('1- Shoulder Lift:', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
            [sg.Text(j1, text_color='Green', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20),key='-JS1-')],
             [sg.Text('2 - Shoulder Pan', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
             [sg.Text('3 - Elbow Flex', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
             [sg.Text('4 -Wrist Pan', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
             [sg.Text('5 - Wrist Lift', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
             [sg.Text('6 - Wrist Rotate', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))],
             [sg.Text('7 - Hand Rotate', text_color='black', background_color='white', justification='left',
                       size=(20, 1), font=('Tw Cen MT', 20))]],
           background_color='white')]
         ]
# Creating the Splash Screen Window Object, to time out after 4 seconds
    splash_window = sg.Window('Splash Screen',
                              splash_layout,
                              no_titlebar=True,
                              keep_on_top=True,
                              background_color='white',
                              grab_anywhere=True).read(timeout=timeout, close=True)

# Creating the Main Window for the GUI, should run indefinitely.
    main_window = sg.Window('Jaco Voice Commander',
                            main_layout, background_color='white',finalize=True)

# While loop that detects events and reads input on the GUI periodically - currently every 100 ms   
    while True:
        event, values = main_window.read(timeout=100)
        if event in (sg.WINDOW_CLOSED, 'Exit'):
            break

        try:
            # what to do once speech event is triggered. Should update text to reflect what Jaco is saying.
            if speak_event.is_set():
                speak = message_queue.get_nowait()
                main_window['-SPEAK-'].update(speak)
                main_window.refresh()
                speak_event.clear()
            # What to do once status event is triggered. Should update the user on what Jaco is currently doing.
            if status_event.is_set():
                status = status_queue.get_nowait()
                main_window['-STATUS-'].update(status)
                main_window.refresh()
                status_event.clear()
            # What to do once joint event is triggered. Should update joint states with values from array.
            if joint_event.is_set():
                joint_angles = status_queue.get_nowait()
                j1= joint_angles[0];j2= joint_angles[1];j3= joint_angles[2];j4= joint_angles[3]
                j5= joint_angles[4];j6= joint_angles[5];j7= joint_angles[6]
                main_window['-JS1-'].update(j1)
                main_window['-JS2-'].update(j2)
                main_window['-JS3-'].update(j3)
                main_window['-JS4-'].update(j4)
                main_window['-JS5-'].update(j5)
                main_window['-JS6-'].update(j6)
                main_window['-JS7-'].update(j7)
                main_window.refresh()
                joint_event.clear()
                
            else:
                pass
        except queue.Empty:
            pass
# audio_processing - The while loop that catches the audio stream, parses it, and searches for usage of keywords or commands.
#                    It also contains the actions the robot and ROS take upon reception of a command, with user feedback.  

def audio_processing():
    # made picovoice variables global so they could be used anywhere in the script
    global recorder, porcupine, rhino
    # Start PVRecorder Object
    recorder.start()
    while True:
        # data from audio stream is being read and processed by porcupine and rhino
        pcm = recorder.read()
        result = porcupine.process(pcm)
        is_finalized = rhino.process(pcm)

        # adding data of array to wav file instantiated earlier
        if wav_file is not None:
            wav_file.writeframes(struct.pack("h" * len(pcm), *pcm))

        # If keyword is detected, perform these actions. Will be commented only once since all actions follow the same structure
        # This first one is for the porcupine object
        if result >= 0:
            # stop the audio stream. Prevents overflow errors
            recorder.stop()
            # prints data to terminal and GUI, and has Jaco verbally affirm to client
            print('[%s] Detected %s' % (str(datetime.now()), keywords[result]))
            rospy.loginfo("Moving to 'Home' State...")
            armstate = 'Home' # set state to be the same as macro previously defined in moveit launch file
            jointrad(armstate) # run the arm movement function with  the specified macro
            print(colored('JACO says:', 'blue') + " At your Service!") # Jaco speech printed to terminal
            speak = 'At your service!' # speech to send to GUI
            message_queue.put(speak) # Putting speech variable in speech queue
            speak_event.set() # Setting event as active to trigger GUI to update
            ps("Audio/At your service 1.wav") # Play audio of Jaco saying this.
            recorder.start() #Start recorder once again to continue reception of commands

        # These are all the commands for the Rhino context I've made. First it detects and prints the intent out to the command line.
        # Then it detects the specific command and does the action needed after that.
        if is_finalized:
            inference = rhino.get_inference()
            if inference.is_understood:
                print("  intent : '%s'" % inference.intent)
                for slot, value in inference.slots.items():
                    print("  %s : '%s'" % (slot, value))
                    direction = value

                recorder.stop()

                if inference.intent == 'move':
                    # Actions to Move Up
                    if direction == 'up':

                        armstate = 'Vertical'
                        print(colored('JACO says:', 'blue') + " Moving Up.")
                        speak = 'Moving Up.'
                        message_queue.put(speak)
                        speak_event.set()
                        ps("Audio/Moving Up 1.wav")

                        status = "Moving to 'Up' State... "
                        rospy.loginfo(status)
                        status_queue.put(status)
                        status_event.set()
                        
                        jointrad(armstate)

                        status = "Current State: 'Up' "
                        rospy.loginfo(status)
                        status_queue.put(status)
                        status_event.set()

                    # Actions to Move Down
                    elif direction == 'down':
                        armstate = 'Down'
                        print(colored('JACO says:', 'blue') + " Moving Down.")
                        speak = 'Moving Down.'
                        message_queue.put(speak)
                        speak_event.set()
                        ps("Audio/Moving Down 1.wav")

                        status = "Moving to 'Down' State... "
                        rospy.loginfo(status)
                        status_queue.put(status)
                        status_event.set()
                        
                        jointrad(armstate)

                        status = "Current State: 'Down' "
                        rospy.loginfo(status)
                        status_queue.put(status)
                        status_event.set()

                    # Actions to Move Left
                    elif direction == 'left':
                        armstate = 'Left'
                        print(colored('JACO says:', 'blue') + " Moving Left.")
                        speak = 'Moving Left.'
                        message_queue.put(speak)
                        speak_event.set()
                        ps("Audio/Moving Left 1.wav")

                        status = "Moving to 'Left' State... "
                        rospy.loginfo(status)
                        status_queue.put(status)
                        status_event.set()
                        
                        jointrad(armstate)

                        status = "Current State: 'Left' "
                        rospy.loginfo(status)
                        status_queue.put(status)
                        status_event.set()

                    # Actions to Move Right
                    elif direction == 'right':
                        armstate = 'Right'
                        print(colored('JACO says:', 'blue') + " Moving Right.")
                        speak = 'Moving Right.'
                        message_queue.put(speak)
                        speak_event.set()
                        ps("Audio/Moving Right 1.wav")
                        
                        status = "Moving to 'Right' State... "
                        rospy.loginfo(status)
                        status_queue.put(status)
                        status_event.set()
                        
                        jointrad(armstate)

                        status = "Current State: 'Right' "
                        rospy.loginfo(status)
                        status_queue.put(status)
                        status_event.set()
                    
                    else:
                        pass
                    recorder.start()
                # Actions to shut down
                if inference.intent == 'shutdown':
                    recorder.stop()

                    print(colored('JACO Says:', 'blue') + " Shutting Down...")
                    speak = 'Shutting Down.'
                    message_queue.put(speak)
                    speak_event.set()
                    ps("Audio/Shutting Down 1.wav")
                    armstate = 'Retract'

                    status = "Moving to 'Retract' State..."
                    rospy.loginfo(status)
                    status_queue.put(status)
                    status_event.set()

                    jointrad(armstate)

                    status = "Shutting Down Program..."
                    rospy.loginfo(status)
                    status_queue.put(status)
                    status_event.set()

                    print(colored('JACO Says:', 'blue') + " Goodbye!")

                    speak = 'Goodbye!'
                    message_queue.put(speak)
                    speak_event.set()
                    ps("Audio/Goodbye 2.wav")
                    ps("Audio/turnoff.wav")
                    exit(0)

# jointrad - Actually moves the arm to the postition specified by the armstate variable.
    
def jointrad(armstate):
    arm.set_named_target(armstate) # sets macro to be triggered within MoveIt!
    plan = arm.go(wait=True) # Executes plan to move robot to pose set by macro

######################################################################################################################################################
# 2. DEFINING VARIABLES AND INSTANTIATING OTHER MODULES NECESSARY FOR OPERATION
######################################################################################################################################################

### THREADS, EVENTS, AND QUEUES ###
#Jaco's Speech Event and Queue for inter-thread communication
message_queue = queue.Queue();speak_event = threading.Event()

#Robot Arm status event and queue
status_queue = queue.Queue();status_event = threading.Event()

#joint states event and queue
joint_queue = queue.Queue();joint_event = threading.Event()

# Create the audio, gui, and joint state listener threads
audio_thread = threading.Thread(target=audio_processing)
gui_thread = threading.Thread(target=GUI_function)
joint_listening_thread = threading.Thread(target=joint_angle_listener)

### ROS ELEMENTS
# Initiating ROS node that this script will operate as
jacovoicer = rospy.init_node('JacoVoicer', anonymous=False)


### MOVEIT! MODULES ###

#Instantiating move group objects. Jaco has 2 move groups, 'arm' and 'gripper'
group_name = 'arm' #better to keep group names as variables for ease later on
arm = moveit_commander.MoveGroupCommander(group_name) # this creates the MoveGroupCommander object also named arm that controls group name
arm.set_max_velocity_scaling_factor(1.0) # Set velocity scaling factor so arm moves at full speed. I believe it is capped at this factor.

# Same thing but with gripper
group2_name = 'gripper'
gripper = moveit_commander.MoveGroupCommander(group2_name)
gripper.set_max_velocity_scaling_factor(1.0)

### PICOVOICE MODULES ###

access_key = '2saeQ0pr4C5VaUWzNDyGAM3vUMli/zt6efotQubsLICInVRXZCRWOA=='  # Required Picovoice Access Key
keyword_paths = ['~/Desktop/Picovoice/Porcupine/Hey-Jaco.ppn']  # Porcupine .ppn file path
context_path = "/home/carlos/Desktop/Picovoice/Rhino/Arm.rhn"  # Rhino .rhn file path

# Starting porcupine and rhino objects 
porcupine = pvporcupine.create(access_key=access_key, keyword_paths=keyword_paths)
rhino = pvrhino.create(access_key=access_key, context_path=context_path)

# Print currently installed versions of software, along with the context's command tree.
print('Porcupine version: %s' % porcupine.version)
print('Rhino version: %s' % rhino.version)
print('Context info: %s' % rhino.context_info)

# Instantiate the PVRecorder object
recorder = PvRecorder(device_index=5, frame_length=porcupine.frame_length)

# Parsing Porcupine wakeword .PPN file
keywords = list()
for x in keyword_paths:
    keyword_phrase_part = os.path.basename(x).replace('.ppn', '').split('_')
    if len(keyword_phrase_part) > 6:
        keywords.append(' '.join(keyword_phrase_part[0:-6]))
    else:
        keywords.append(keyword_phrase_part[0])

# Audio Stream needs to be stored in a folder while it is being used. 
# For now each instance of the program overwrites the previous audio recording but can probably be changed to provide logs.
wav_file = wave.open('Stream/jacoaudio.wav', "w")
wav_file.setnchannels(1)
wav_file.setsampwidth(2)
wav_file.setframerate(16000)

# Enables faulthandler - more verbose error messages - sometimes useful to me but not necessary for the script to function
faulthandler.enable()

######################################################################################################################################################
# 2. MAIN SECTION OF CODE - STARTUP ALL THREADS AND FUNCTIONS
######################################################################################################################################################

if __name__ == '__main__':
    try:
        gui_thread.start() # Start GUI Thread
        ps("Audio/boot.wav") #Startup Sound
        joint_listening_thread.start() # Start Joint Listener Thread

        # Saying Hello
        print(colored('JACO says:', 'blue') + " Hello There!")
        speak = 'Hello there!'
        message_queue.put(speak)
        speak_event.set()
        ps("Audio/Hello There 2.wav")

        # Introducing Himself
        print(colored('JACO says:', 'blue') + " I am Jaco, your robotic assistant.")
        speak = 'I am Jaco, your robotic assistant.'
        message_queue.put(speak)
        speak_event.set()
        ps("Audio/I am Jaco.wav")

        # Moving to Home position to start from a neutral point every time
        status = "Moving to 'Home' State... "
        rospy.loginfo(status)
        status_queue.put(status)
        status_event.set()

        armstate = 'Home'
        jointrad(armstate)

        status = "Current State: 'Home'"
        rospy.loginfo(status)
        status_queue.put(status)
        status_event.set()

        # Letting user know Jaco is ready to go
        print(colored('JACO says:', 'blue') + " I'm Listening...")
        speak = "I'm listening..."
        message_queue.put(speak)
        speak_event.set()
        ps("Audio/Listening.wav")

        # Print audio device used for debugging purposes
        print('Using device: %s' % recorder.selected_device)
        # Start Speech processing Thread
        audio_thread.start()

    except rospy.ROSInterruptException:
        pass
