#!/usr/bin/env python3
import threading
from voiceline import VoiceLine
from robotmove import RobotMove
from time import sleep
from shared_instances import zc
from picovoicer import PicoVoicer

#ROS_Voicer.py --- Main Script for running voice command app. No GUI involved to reduce overhead.
# Carlos Mella-Rijo
# Dynamic Networks and Control Laboratory
# University of Texas at Arlington, Department of Electrical Engineering

home_pose = RobotMove.home_pose
pvr = PicoVoicer()

def main():
    try:
        #Main Function.
        VoiceLine.boot()
        sleep(1)
        VoiceLine.greeting()
        
        print("Starting camera feed thread...")
        camera_feed_thread = threading.Thread(target=zc.cam_loop)
        camera_feed_thread.start()
        print("Camera feed thread started.")
        print("Starting audio processing thread...")
        
        audio_thread = threading.Thread(target=pvr.audio_processing)
        audio_thread.start()
        print("Audio processing thread started.")

        RobotMove.home()
        RobotMove.gripper_client([0,0,0])
        VoiceLine.listening()
        
    except KeyboardInterrupt:
        VoiceLine.shutdown()
        zc.stop_cam_loop()
        camera_feed_thread.join()
        audio_thread.join()
        print("closing...")
        exit(0)
    pass

if __name__ == "__main__":
    main()
