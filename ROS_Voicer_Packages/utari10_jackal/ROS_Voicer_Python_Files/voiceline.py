from playsound import playsound as ps
from termcolor import colored

# voiceline.py - Class that contains all of the Jackal's speech
# Carlos Mella-Rijo
# Dynamic Networks and Control Laboratory
# University of Texas at Arlington, Department of Electrical Engineering

class VoiceLine:
    def greeting():
        print(colored('JACKSON says:', 'blue') + "Hello There!")
        ps('Audio/01-Greeting.wav')
        print(colored('JACKSON says:', 'blue') + "I am Jackson, your Robotic Assistant.")
        ps('Audio/02-Intro.wav')
        pass
    def listening():
        print(colored('JACKSON says:', 'blue') + "I'm listening...")
        ps('Audio/03-Listening.wav')
        pass
    def up():
        print(colored('JACKSON says:', 'blue') + "Moving Up...")
        ps('Audio/04-MovingUp.wav')
        pass

    def down():
        print(colored('JACKSON says:', 'blue') + "Moving Down...")
        ps('Audio/05-MovingDown.wav')
        pass

    def right():
        print(colored('JACKSON says:', 'blue') + "Moving Right...")
        ps('Audio/07-MovingRight.wav')
        pass

    def left():
        print(colored('JACKSON says:', 'blue') + "Moving Left...")
        ps('Audio/06-MovingLeft.wav')
        pass

    def forward():
        print(colored('JACKSON says:', 'blue') + "Moving Forward...")
        ps('Audio/08-MovingForward.wav')
        pass

    def back():
        print(colored('JACKSON says:', 'blue') + "Moving Back...")
        ps('Audio/09-MovingBackward.wav')
        pass

    def shutdown():
        print(colored('JACKSON says:', 'blue') + "Shutting down. Goodbye!")
        ps('Audio/10-ShuttingDown.wav')
        ps('Audio/11-Goodbye.wav')
        pass

    def find():
        print(colored('JACKSON says:', 'blue') + "Looking for bottle...")
        ps('Audio/12-Looking.wav')
        pass
    
    def found():
        print(colored('JACKSON says:', 'blue') + "Found a bottle!")
        ps('Audio/13-Found.wav')
        pass

    def moving():
        print(colored('JACKSON says:', 'blue') + "Moving to bottle location...")
        ps('Audio/14-MovingTo.wav')
        pass

    def grab():
        print(colored('JACKSON says:', 'blue') + "Grabbing...")
        ps('Audio/15-Grabbing.wav')
        pass
    
    def release():
        print(colored('JACKSON says:', 'blue') + "Releasing...")
        ps('Audio/16-Releasing.wav')
        pass

    def tell_me(part):
        print(colored('JACKSON says:', 'blue') + "Of course!")
        ps('Audio/17-OfCourse.wav')

        if part == 'camera':
            print(colored('JACKSON says:', 'blue') + "Installed on my chassis is a ZED Stereo camera.")
            ps('Audio/18-ZedCam-1.wav')
            print("It calculates the depth at various points in an image the same way your eyes do:")
            print("by calculating the difference in position between the same objects in the image of the left and right cameras.")
            ps('Audio/19-ZedCam-2.wav')
            print("Various transformations are done by the ZED API to convert pixel values to 3-D coordinates in a point cloud.")
            ps('Audio/20-ZedCam-3.wav')
            print("I can then use YOLO to detect objects in the camera frame, gather the equivalent point cloud values, and pass the object's coordinates to the arm. ")
            ps('Audio/21-ZedCam-4.wav')
            pass
        if part == 'wheels':
            print(colored('JACKSON says:', 'blue') + "My wheels are part of the Jackal UGV by Clearpath Robotics.")
            ps('Audio/22-Jackal-1.wav')
            print(" You can command me to move myself if you ask me to 'Move' myself forward or backward.")
            ps('Audio/23-Jackal-2.wav')
            print("The units must be in centimeters.")
            ps('Audio/31-DistanceUnits.wav')
            print("I can also turn left or right based on an angle if you ask me to 'Turn' myself.")
            ps('Audio/24-Jackal-3.wav')
            print("The angle units are degrees.")
            ps('Audio/32-AngleUnits.wav')
            pass
        if part == 'arm':
            print(colored('JACKSON says:', 'blue') + "My arm is a Jaco2 arm by Kinova Robotics.")
            ps('Audio/25-JacoArm-1.wav')
            print("It was originally created as an assisted living device for persons with disabilities.")
            ps('Audio/26-JacoArm-2.wav')
            print(" You can command me to move my arm in cartesian space if you ask me to 'Move' my arm.")
            ps('Audio/27-JacoArm-3.wav')
            print(" I understand the directions forward, back, left, right, up, and down.")
            ps('Audio/28-JacoArm-4.wav')
            print("The units must be in centimeters.")
            ps('Audio/31-DistanceUnits.wav')
            print("I can also turn left or right based on an angle if you ask me to 'Turn' my arm.")
            ps('Audio/29-JacoArm-5.wav')
            print("The angle units are degrees.")
            ps('Audio/32-AngleUnits.wav')
            print('I can also detect and move to a bottle, if you ask me to "find" a bottle.')
            ps('Audio/30-JacoArm-6.wav')
            pass
        pass
    def cant_do():
        print(colored('JACKSON says:', 'blue') + "Can't do that, boss.")
        ps('Audio/33-CantDoThat.wav')
        pass
    def misunderstand():
        print(colored('JACKSON says:', 'blue') + "I didn't catch that.")
        ps('Audio/34-DidntCatch.wav')
        print('May you please say it again?')
        ps('Audio/35-SayAgain.wav')
        pass
    def error_occured():
        print(colored('JACKSON says:', 'blue') + "An error has occured!")
        ps('Audio/36-Error.wav')
        pass
    def confirm():
        print(colored('JACKSON says:', 'blue') + "As you wish.")
        ps('Audio/38-AsYouWish.wav')
        pass
    def boot():
        ps('Audio/Boot.wav')
        pass
    def connect():
        ps('Audio/Connect.wav')
        pass
    def disconnect():
        ps('Audio/Disconnect.wav')
        pass
    def turnoff():
        ps('Audio/ShutDown.wav')
