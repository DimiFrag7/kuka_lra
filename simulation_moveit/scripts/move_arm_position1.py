#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math
from std_srvs.srv import Empty

def call_service(service_name):
    rospy.wait_for_service(service_name)
    try:
        service_proxy = rospy.ServiceProxy(service_name, Empty)
        service_proxy()
        rospy.loginfo(f"Successfully called {service_name}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    rospy.init_node('move_arm_and_call_services', anonymous=True)

    move_group = moveit_commander.MoveGroupCommander("arm_manipulator")

    first_pose = Pose()
    first_pose.position.x = 0.3
    first_pose.position.y = 0
    first_pose.position.z = 0.8
    
    roll_deg = 1
    pitch_deg = 90
    yaw_deg = 0
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)
    quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
    first_pose.orientation.x = quaternion[0]
    first_pose.orientation.y = quaternion[1]
    first_pose.orientation.z = quaternion[2]
    first_pose.orientation.w = quaternion[3]
   
    move_group.set_pose_target(first_pose)
    move_group.set_planning_time(20)
    move_group.set_max_velocity_scaling_factor(0.5)
    move_group.set_max_acceleration_scaling_factor(0.5)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.sleep(2)

    on_services = [
        '/kuka_grasp/on',
        '/kuka_grasp1/on',
        '/kuka_grasp2/on',
        '/kuka_grasp3/on',
        '/kuka_grasp4/on',
        '/kuka_grasp5/on',
        '/kuka_grasp6/on',
        '/kuka_grasp7/on',
        '/kuka_grasp8/on'
    ]

    for service in on_services:
        call_service(service)

    rospy.sleep(2)

    # Move the arm to the second target position
    second_pose = Pose()
    second_pose.position.x = 0.3
    second_pose.position.y = 0
    second_pose.position.z = 0.8 + 0.3
    # Use the same orientation as the first pose
    second_pose.orientation = first_pose.orientation
    move_group.set_pose_target(second_pose)
    move_group.set_planning_time(20)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Rotate the arm
    joint_values = move_group.get_current_joint_values()
    joint_values[0] += math.radians(120)
    move_group.set_joint_value_target(joint_values)
    move_group.set_planning_time(20)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    red_pose = Pose()
    red_pose.position.x = -0.4
    red_pose.position.y = -0.3
    red_pose.position.z = 0.8 + 0.3
    # Define orientation in degrees
    roll_deg = 179
    pitch_deg = 90
    yaw_deg = 0
    # Convert degrees to radians
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)
    # Convert RPY angles to quaternion
    quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
    red_pose.orientation.x = quaternion[0]
    red_pose.orientation.y = quaternion[1]
    red_pose.orientation.z = quaternion[2]
    red_pose.orientation.w = quaternion[3]
    move_group.set_pose_target(red_pose)
    move_group.set_planning_time(20)
    move_group.set_max_velocity_scaling_factor(0.08)
    move_group.set_max_acceleration_scaling_factor(0.05)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    redplace_pose = move_group.get_current_pose().pose
    redplace_pose.position.z -= 0.15
    move_group.set_pose_target(redplace_pose)
    move_group.set_planning_time(20)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.sleep(2)


    off_services = [
        '/kuka_grasp/off',
        '/kuka_grasp1/off',
        '/kuka_grasp2/off',
        '/kuka_grasp3/off',
        '/kuka_grasp4/off',
        '/kuka_grasp5/off',
        '/kuka_grasp6/off',
        '/kuka_grasp7/off',
        '/kuka_grasp8/off'
    ]

    for service in off_services:
        call_service(service)
      
    move_group.set_named_target("home")
    move_group.set_max_velocity_scaling_factor(0.5)  # 10% of max velocity
    move_group.set_max_acceleration_scaling_factor(0.5)  # 10% of max acceleration

    plan = move_group.go(wait=True)

    move_group.stop()
    move_group.clear_pose_targets()  




if __name__ == "__main__":
    main()



