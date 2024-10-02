#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def joint_states_callback(msg):
    # Print the joint names and their corresponding positions in radians
    for name, position in zip(msg.name, msg.position):
        rospy.loginfo(f"Joint: {name}, Position (radians): {position}")

def joint_state_listener():
    rospy.init_node('joint_state_listener', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        joint_state_listener()
    except rospy.ROSInterruptException:
        pass
