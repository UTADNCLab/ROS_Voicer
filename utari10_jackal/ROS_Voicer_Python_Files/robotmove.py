import geometry_msgs
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
from math import pi
import rospy
from kinova_msgs.msg import PoseVelocity, JointVelocity
from geometry_msgs.msg import Pose
import kinova_msgs
from kinova_msgs.srv import HomeArm, HomeArmRequest, HomeArmResponse
import actionlib
import std_msgs.msg

# Classes for movement of arm and jackal
class RobotMove:
    rospy.init_node('Robot_Move', anonymous=True)
    #Default Orientation variables - i convert r-p-y to quarternion for simplicity 
    roll = 2.1472
    pitch = 0.122
    yaw = -0.1384
    pose_goal = Pose()
    quarternion = quaternion_from_euler(roll,pitch,yaw)
    pose_goal.orientation.x = quarternion[0]
    pose_goal.orientation.y = quarternion[1]
    pose_goal.orientation.z = quarternion[2]
    pose_goal.orientation.w = quarternion[3]
    linear_velocity = 0.1
    angular_velocity = 0.3
    # Home cartesian pose for arm
    home_pose = [0.208, -0.251, 0.496]

    # Gripper Joint values 
    grip_open = [0.35, 0, 0.35, 0, 0.35, 0]
    grip_close = [1.15, 0, 1.15, 0, 1.15, 0]

    # Wait for the service to become available
    rospy.wait_for_service('/j2n6s300_driver/in/home_arm')

    def home():
        # Create a service proxy
        home_service_proxy = rospy.ServiceProxy('/j2n6s300_driver/in/home_arm', HomeArm)
        # Create a service request
        request = HomeArmRequest()
        # Set values in the request as needed
        try:
            response = home_service_proxy(request)
            # Process the response
            print("Home Arm service call successful!")
            print("Response: {}".format(response))
        except rospy.ServiceException as e:
            print("Home Arm service call failed: {}".format(e))
        pass

    def pub_twist_msg_jackal(linear_velocity, angular_velocity, pub_twist, duration):
        # Create a Twist message object
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            pub_twist.publish(twist_msg)
            rate.sleep()

    def pub_twist_msg_arm(x, y, z, rx, ry, rz, pub_twist_arm, duration):
        twist_msg = PoseVelocity()
        twist_msg.twist_linear_x = x
        twist_msg.twist_linear_y  = y
        twist_msg.twist_linear_z  = z
        twist_msg.twist_angular_x  = rx
        twist_msg.twist_angular_y = ry
        twist_msg.twist_angular_z = rz
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(100)

        print(twist_msg)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            print("publishing...")
            pub_twist_arm.publish(twist_msg)
            rate.sleep()
        print("Done.")

    def pub_twist_msg_arm_joint(joint_1, pub_twist_arm_joint, duration):
        twist_msg = JointVelocity()
        twist_msg.joint1 = joint_1
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(100)
        print(twist_msg)
        print("publishing...")

        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            pub_twist_arm_joint.publish(twist_msg)
            rate.sleep()
        print("Done.")

    def cartesian_pose_client(position, orientation):
        """Send a cartesian goal to the action server."""
        action_address = 'j2n6s300_driver/pose_action/tool_pose'
        client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=('j2n6s300_link_base'))
        goal.pose.pose.position = geometry_msgs.msg.Point(
            x=position[0], y=position[1], z=position[2])
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

        client.send_goal(goal)

        if client.wait_for_result(rospy.Duration(10.0)):
            return client.get_result()
        else:
            client.cancel_all_goals()
            print('        the cartesian action timed-out')
            return None
        
    def gripper_client(finger_positions):
        """Send a gripper goal to the action server."""
        action_address = '/j2n6s300_driver/fingers_action/finger_positions'

        client = actionlib.SimpleActionClient(action_address,
                                            kinova_msgs.msg.SetFingersPositionAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float(finger_positions[0])
        goal.fingers.finger2 = float(finger_positions[1])
        goal.fingers.finger3 = float(finger_positions[2])
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(5.0)):
            print("KINOVA ARM GRIPPER STATE CHANGED")
            return client.get_result()
        else:
            client.cancel_all_goals()
            rospy.logwarn('        the gripper action timed-out')
            return None