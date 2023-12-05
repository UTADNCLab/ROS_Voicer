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

# ROS_Voicer and utari10_jackal Package

This section covers all that is required to Run the utari10_jackal package and the ROS_Voicer script within it. More in-depth tutorials will be available in the wiki, and will be linked accordingly.
[ utari10_jackal README.md](ROS_Voicer_Packages/utari10_jackal/README.md)

# TurtleBot Voice Commander

This section covers all that is required to run the TurtleBot Voice Commander program. It will also cover how to use the program. Just like for the ROS_Voicer package, more in-depth tutorials will be available in the wiki, and will be linked accordingly.
[Turtlebot_README.md](TurtleBot_Voice/Turtlebot_README.md)

# Jaco Voice Commander

This section will cover all that is required to run the Jaco Voice Commander program. It will also cover how to use the program. Just like for the Turtlebot and ROS_Voicer, more in-depth tutorials will be available in the wiki, and will be linked accordingly.
[Jaco_README.md](Jaco_Voice/Jaco_README.md)
