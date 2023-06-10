import sys
import rospy
import moveit_commander
import faulthandler
import geometry_msgs

# debug.py - testing movement methods available with moveit_commander
faulthandler.enable()
rospy.init_node('foo', anonymous=False)

group_name = 'arm'
move_group = moveit_commander.MoveGroupCommander(group_name)

# gripper move_group2
group2_name = 'gripper'
move_group2 = moveit_commander.MoveGroupCommander(group2_name)
move_group2.set_max_velocity_scaling_factor(1.0)


move_group.set_max_velocity_scaling_factor(1.0)

joint_angles = [1, 1, 1, 1, 1, 1]  # Replace with the desired joint angles
move_group.set_joint_value_target(joint_angles)
plan = move_group.go(wait=True)

# Set the named target state
named_target = "Vertical"  # Replace with the name of the desired preset state
move_group.set_named_target(named_target)
print("moving to pose 3...")
plan = move_group.go(wait=True)
print("Finished!!!!!")

print("Opening Gripper...")
named_target2 = "Open"
move_group2.set_named_target(named_target2)
plan = move_group2.go(wait=True)
print("Finished!!!!!")

named_target2 = "Close"
move_group2.set_named_target(named_target2)
print("Closing Gripper...")
plan = move_group2.go(wait=True)
print("Finished!!!!!")

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.04
pose_goal.position.y = 0.01
pose_goal.position.z = 0.04
move_group.set_pose_target(pose_goal)

plan = move_group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()




