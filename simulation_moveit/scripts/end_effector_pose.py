#!/usr/bin/env python3

import rospy
import moveit_commander

def get_end_effector_pose():
    rospy.init_node('get_end_effector_pose_node', anonymous=True)
    
    move_group = moveit_commander.MoveGroupCommander("arm_manipulator")
    
    current_pose = move_group.get_current_pose().pose
    
    rospy.loginfo("Current End Effector Pose:")
    rospy.loginfo(f"Position: x={current_pose.position.x}, y={current_pose.position.y}, z={current_pose.position.z}")
    rospy.loginfo(f"Orientation: x={current_pose.orientation.x}, y={current_pose.orientation.y}, z={current_pose.orientation.z}, w={current_pose.orientation.w}")
    
    return current_pose

if __name__ == '__main__':
    try:
        get_end_effector_pose()
    except rospy.ROSInterruptException:
        pass
