#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander

rospy.init_node('move_arm_position_node', anonymous=True)

move_group = MoveGroupCommander("manipulator")

target_positions = [1.0, 0.5, -1.0, -1.5, -1.0, 0.0]
move_group.set_joint_value_target(target_positions)

plan = move_group.go(wait=True)

move_group.stop()
move_group.clear_pose_targets()

rospy.sleep(3)

move_group.set_named_target("home")
plan = move_group.go(wait=True)

move_group.stop()
move_group.clear_pose_targets()
