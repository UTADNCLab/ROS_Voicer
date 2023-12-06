import wave, struct
import pvporcupine, pvrhino
from math import radians
from pvrecorder import PvRecorder
from robotmove import RobotMove
from voiceline import VoiceLine
from zedcam import ZedCam
import rospy
from geometry_msgs.msg import Twist
from kinova_msgs.msg import PoseVelocity, JointVelocity
from shared_instances import zc

# picovoicer.py - class that contains all audio processing done by PicoVoice Modules and calls of other classes to facilitate movement
# Carlos Mella-Rijo
# Dynamic Networks and Control Laboratory
# University of Texas at Arlington, Department of Electrical Engineering
class PicoVoicer:
    rate = rospy.Rate(1)
    def audio_processing(self):
        #Function for detecting voice commands with rhino context. Will be a separate thread in main script.
        linear_velocity = RobotMove.linear_velocity
        angular_velocity = RobotMove.angular_velocity
        grip_open = RobotMove.grip_open
        grip_close = RobotMove.grip_close
        home = RobotMove.home_pose

        # Create publishers for the specified topics and message types
        pub_twist = rospy.Publisher('/twist_marker_server/cmd_vel', Twist, queue_size=10)
        pub_twist_arm = rospy.Publisher('/j2n6s300_driver/in/cartesian_velocity', PoseVelocity, queue_size=10)
        pub_twist_arm_joint = rospy.Publisher('/j2n6s300_driver/in/joint_velocity',JointVelocity, queue_size=10 )
        
        while True:
            # data from audio stream is being read and processed by porcupine and rhino
            pcm = recorder.read()
            is_finalized = rhino.process(pcm)
            # adding data of array to wav file instantiated earlier
            if wav_file is not None:
                wav_file.writeframes(struct.pack("h" * len(pcm), *pcm))
            
            # These are all the commands for the Rhino context I've made. First it detects and prints the intent out to the command line.
            # Then it detects the specific command and does the action needed after that.
            if is_finalized:
                inference = rhino.get_inference()
                if inference.is_understood:
                    print('{')
                    print("  intent : '%s'" % inference.intent)
                    print('  slots : {')
                    for slot, value in inference.slots.items():
                        print("    %s : '%s'" % (slot, value))
                    print('  }')
                    print('}')

                    recorder.stop()
                    if inference.intent =='move':
                        
                        # moveit_commander takes distance values in meters, so distance must be converted
                        distance = float(inference.slots['distance']) * 0.01 
                        print(f"Understood distance:{distance}")
                        body_part = inference.slots['body_part']

                        if body_part == 'arm':
                            duration = float(distance) / linear_velocity
                            print(f"Duration: {distance} m / {linear_velocity} m/s = {duration} s")
                            direction = inference.slots['direction']
                            
                            if direction == 'up':
                                VoiceLine.up()
                                RobotMove.pub_twist_msg_arm(0,0,linear_velocity,0,0,0,pub_twist_arm,duration)
                                pass
                            
                            elif direction == 'down':
                                VoiceLine.down()
                                RobotMove.pub_twist_msg_arm(0,0,-linear_velocity,0,0,0,pub_twist_arm,duration)
                                pass
                            elif direction == 'left':
                                VoiceLine.left()
                                RobotMove.pub_twist_msg_arm(linear_velocity,0,0,0,0,0,pub_twist_arm,duration)
                                pass
                            elif direction == 'right':
                                VoiceLine.right()
                                RobotMove.pub_twist_msg_arm(-linear_velocity,0,0,0,0,0,pub_twist_arm,duration)
                                pass
                            elif direction == 'forward':
                                VoiceLine.forward()
                                RobotMove.pub_twist_msg_arm(0,-linear_velocity,0,0,0,0,pub_twist_arm,duration)
                                pass
                            elif direction == 'back':
                                VoiceLine.back()
                                RobotMove.pub_twist_msg_arm(0,linear_velocity,0,0,0,0,pub_twist_arm,duration)
                                pass
                            pass

                        elif body_part == 'self':
                            # messages are in velocity.
                            duration = float(distance) / linear_velocity
                            direction = inference.slots['direction']

                            if direction == 'forward':
                                VoiceLine.forward()
                                RobotMove.pub_twist_msg_jackal(linear_velocity,0,pub_twist, duration)
                                pass
                            elif direction == 'back':
                                VoiceLine.back()
                                RobotMove.pub_twist_msg_jackal(-linear_velocity,0,pub_twist, duration)
                                pass
                            else:
                                VoiceLine.cant_do()
                                pass
                        pass

                    elif inference.intent =='turn':
                        body_part = inference.slots['body_part']
                        value =  float(inference.slots['degrees'])
                        # moveit_commander takes angle values in radians, so angle input must be converted
                        angle = radians(value)
                        direction = inference.slots['direction']

                        if body_part =='arm':
                            angular_velocity_arm = 20 # deg/s
                        
                            duration = value / angular_velocity_arm
                            print(f"Duration: {value} deg / {angular_velocity_arm}(deg/s) = {duration} s ")
                            
                            if direction == 'left':
                                VoiceLine.left()
                                RobotMove.pub_twist_msg_arm_joint(- angular_velocity_arm,pub_twist_arm_joint,duration)
                                pass
                            elif direction == 'right':
                                VoiceLine.right()
                                RobotMove.pub_twist_msg_arm_joint(angular_velocity_arm,pub_twist_arm_joint,duration)
                                pass
                            else:
                                VoiceLine.cant_do()
                                pass
                            pass

                        elif body_part =='self':
                            duration = angle / angular_velocity
                            if direction == 'left':
                                VoiceLine.left()
                                RobotMove.pub_twist_msg_jackal(0,angular_velocity,pub_twist,duration)
                                pass
                            elif direction == 'right':
                                VoiceLine.right()
                                RobotMove.pub_twist_msg_jackal(0,-angular_velocity, pub_twist, duration)
                                pass
                            else:
                                VoiceLine.cant_do()
                                pass
                            pass
                        pass
                    elif inference.intent =='go_home':
                        VoiceLine.confirm()
                        RobotMove.home()
                        pass
                    elif inference.intent =='grab':
                        VoiceLine.grab()
                        RobotMove.gripper_client([5000,5000,5000])
                        pass
                    elif inference.intent =='release':
                        VoiceLine.release()
                        RobotMove.gripper_client([0,0,0])
                        pass
                    elif inference.intent =='find':
                        VoiceLine.find()
                        image_ocv = ZedCam.image_ocv
                        PicoVoicer.rate.sleep()
                        ZedCam.detector(image_ocv)
                        pass
                    elif inference.intent =='tell_me':
                        part = inference.slots['part']
                        VoiceLine.tell_me(part)
                        pass
                    elif inference.intent =='shutdown':
                        VoiceLine.shutdown()
                        zc.stop_cam_loop()
                        VoiceLine.turnoff()
                        print("Closing...")
                        exit(0)
                    else:
                        VoiceLine.misunderstand()
                        pass
                    recorder.start()
            pass

access_key = #Insert Picovoice Access Key Here
context_path = "Picovoice/Rhino/arm_move_en_linux.rhn"  # Rhino .rhn file path
keyword_path = ["Picovoice/Porcupine/Hey-Jackson_en_linux_v3_0_0.ppn"] #Porcupine .ppn file path

# Starting rhino objects 
rhino = pvrhino.create(access_key=access_key, context_path=context_path)
porcupine =pvporcupine.create(access_key, keyword_paths= keyword_path)
devices = PvRecorder.get_available_devices()
for i, device in enumerate(PvRecorder.get_available_devices()):
    print('Device %d: %s' % (i, device))
# Instantiate the PVRecorder object
recorder = PvRecorder(device_index=-1, frame_length=rhino.frame_length)
# Start PVRecorder Object
recorder.start()
# Audio Stream needs to be stored in a folder while it is being used. 
# For now each instance of the program overwrites the previous audio recording but can probably be changed to provide logs.
wav_file = wave.open('Stream/jacoaudio.wav', "wb")
wav_file.setnchannels(1)
wav_file.setsampwidth(2)
wav_file.setframerate(16000)

