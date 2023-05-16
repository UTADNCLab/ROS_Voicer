#! /usr/bin/env python
#!/usr/bin/python -tt
import os, struct, wave, pvporcupine, pvrhino, sys, roslaunch, copy, rospy, subprocess, faulthandler, threading
from datetime import datetime, time
from pvrecorder import PvRecorder
from std_msgs.msg import Float64
import numpy as np
from playsound import playsound as ps
import tkinter as tk
from tkinter import *

class PicovoiceThread(Thread):

    def jointrad(state):
        pub3.publish(state[2]);pub2.publish(state[1]);pub1.publish(state[0]);
        pub4.publish(state[3]);pub5.publish(state[4]);
        rate.sleep()
        rospy.loginfo("Joint Angle is: [%1.2f %1.2f %1.2f %1.2f %1.2f] ",
                    state[0],state[1],state[2],state[3],state[4])
    
    def _wake_word_callback(self):
        self._time_label.configure(fg='red')

    def _inference_callback(self, inference):
        self._time_label.configure(fg='black')

        if result >= 0:
                recorder.stop()
                print('[%s] Detected %s' % (str(datetime.now()), keywords[result]))
                print("Moving to 'NEUTRAL'State...")
                state = np.array([-1.8,-1.4,-1,1.5,0])
                jointrad(state)
                ps('listening.wav')
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
                            ps('Audio/up.wav')
                            rospy.loginfo("Moving to 'UP' state...")
                            jointrad(state)
                            ps('Audio/tada.wav')
                        #move to Down state
                        elif direction =='down':
                            state = np.array([-1.8,-1.7,1.5,1.5,0])
                            ps('Audio/down.wav')
                            print("Moving Down")
                            rospy.loginfo("Moving to 'DOWN' state...")
                            jointrad(state)
                            ps('Audio/tada.wav')
                        #move to Left state
                        elif direction =='left':
                            state = np.array([-1,-1.4,-1,1.5,0])
                            ps('Audio/left.wav')
                            rospy.loginfo("Moving to 'LEFT' state...")
                            jointrad(state)
                            ps('Audio/tada.wav')
                        #move to Right state
                        elif direction =='right':
                            state = np.array([-2.6,-1.4,-1,1.5,0])
                            ps('Audio/right.wav')
                            rospy.loginfo("Moving to 'RIGHT' state...")
                            jointrad(state)
                            ps('Audio/tada.wav')
                        else:
                            pass
                    #start recorder to continue listening.
                    recorder.start()
                    
                    # wave motion
                    if inference.intent == 'sayHi':
                        recorder.stop()
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
                        rate.sleep()

                        state = np.array([-1.7,-1.7,1.5,1.5,0])
                        rospy.loginfo("Moving to 'DOWN' state...")
                        jointrad(state)
                        rate.sleep()
                        rate.sleep()

                        ps('Audio/grabbing.wav')
                        state = np.array([-1.7,-1.7,1.5,1.5,-2.2])
                        rospy.loginfo("Moving to 'CLOSING' state...")
                        jointrad(state)
                        
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
                        

                        ps('Audio/releasing.wav')
                        rospy.loginfo("Moving to 'Opening' state...")
                        state = np.array([-2.6,-1.7,1.5,1.5,0])
                        jointrad(state)
                        rate.sleep()
                        rate.sleep()

                        state = np.array([-1.8,-1.4,-1,1.5,0])
                        jointrad(state)
                        ps('Audio/tada.wav')
                        recorder.start()

                    if inference.intent == 'shutdown':
                       recorder.stop()
                       print("Shutting Down. Goodbye!")
                       ps('Audio/shutdown.wav')
                       print("Resetting to 'NEUTRAL'State...")
                       state = np.array([-1.8,-1.4,-1,1.5,0])
                       jointrad(state)
                       ps('Audio/tada.wav')
                       launcher = subprocess.call(['/bin/bash', '-i', '-c', killall],stdout=True,stderr=True, text=True)
                       exit(0)

    def run(self):
        porcupine = None
        rhino = none
        recorder = None

        # declaring args for picovoice modules before using them
        access_key = 'BKZ6QIyCZPIirHUNrqCmxR8mwVJHIPSGFv22P2/rFGpqJilSGLmf/g=='# Required Picovoice Access Key
        keyword_paths = ['~/Desktop/Picovoice/Porcupine/Hey-Turtle-bot.ppn'] # Porcupine .ppn file path
        context_path = "/home/carlos/Desktop/Picovoice/Rhino/Arm.rhn" # Rhino .rhn file path

        try:
            # Creating Porcupine wake word engine and Rhino speech-to-intent engine instances
            porcupine = pvporcupine.create(access_key=access_key,keyword_paths=keyword_paths) 
            rhino = pvrhino.create(access_key=access_key,context_path=context_path)
            
            # Printing Module Versions and Context information for clarity
            print('Porcupine version: %s' % porcupine.version)
            print('Rhino version: %s' % rhino.version)
            print('Context info: %s' % rhino.context_info)

            recorder = PvRecorder(device_index=-1, frame_length=pv.frame_length)
            recorder.start()

            self._is_ready = True

            # Parses keywords from porcupine
            keywords = list()
            for x in keyword_paths:
                keyword_phrase_part = os.path.basename(x).replace('.ppn', '').split('_')
                if len(keyword_phrase_part) > 6:
                    keywords.append(' '.join(keyword_phrase_part[0:-6]))
                else:
                    keywords.append(keyword_phrase_part[0])

            while not self._stop:
                pcm = recorder.read()
                pv.process(pcm)
        finally:
            if recorder is not None:
                recorder.delete()

            if pv is not None:
                pv.delete()

        self._is_stopped = True

    def is_ready(self):
        return self._is_ready

    def stop(self):
        self._stop = True

    def is_stopped(self):
        return self._is_stopped
    
def main():
    # Command to start launch file with ros as a subprocess
    command = 'gnome-terminal --tab -- bash -c "cd ~/catkin_ws/src/pincher_arm/pincher_arm_bringup/launch/ && ls -l &&roslaunch arm.launch;exec bash"'
    # Command to kill all processes (used later but declared here)
    killall = 'gnome-terminal --tab -- bash -c "killall -9 roslaunch && killall -9 rosmaster && killall -9 rosout"'
    launcher = subprocess.call(['/bin/bash', '-i', '-c', command],stdout=True,stderr=True, text=True)

    # Enables 'faulthandler' - dumps Python tracebacks explicitly, on a fault, after a timeout, or on a user signal. 
    faulthandler.enable()

    # PicoPublisher - the ros publishing node - publishes all joint states
    rospy.init_node('PicoPublisher', anonymous=True)
    rate = rospy.Rate(0.5) # 1/2 hz update rate for state publishing
    pub1 = rospy.Publisher('/arm_shoulder_pan_joint/command', Float64, queue_size=10) # Servo 1 - Shoulder Pan
    pub2 = rospy.Publisher('/arm_shoulder_lift_joint/command', Float64, queue_size=10)# Servo 2 - Shoulder Lift
    pub3 = rospy.Publisher('/arm_elbow_flex_joint/command', Float64, queue_size=10)   # Servo 3 - Elbow
    pub4 = rospy.Publisher('/arm_wrist_flex_joint/command', Float64, queue_size=10)   # Servo 4 - Wrist
    pub5 = rospy.Publisher('/gripper_joint/command', Float64, queue_size=10)          # Servo 5 - Gripper

    # Printing Module Versions and Context information for clarity
    print('Porcupine version: %s' % porcupine.version)
    print('Rhino version: %s' % rhino.version)
    print('Context info: %s' % rhino.context_info)

    window = tk.Tk()
    window.title('Picovoice Demo')
    window.minsize(width=400, height=200)

    time_label = tk.Label(window, text='00 : 00 : 00', font=('Ubuntu', 48))
    time_label.pack(fill=tk.BOTH, pady=90)

    picovoice_thread = PicovoiceThread(time_label, args.access_key)

    def on_close():
        picovoice_thread.stop()
        while not picovoice_thread.is_stopped():
            pass
        window.destroy()

    window.protocol('WM_DELETE_WINDOW', on_close)

    picovoice_thread.start()
    while not picovoice_thread.is_ready():
        pass

    window.mainloop()

if __name__ == '__main__':
    main()