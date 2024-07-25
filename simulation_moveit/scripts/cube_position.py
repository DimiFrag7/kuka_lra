#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates

def model_states_callback(msg):
    try:
        index = msg.name.index("cube")
        position = msg.pose[index].position
        rospy.loginfo("Cube position: x={}, y={}, z={}".format(position.x, position.y, position.z))
    except ValueError:
        rospy.logwarn("Cube not found in the model states.")

def main():
    rospy.init_node("cube_position_printer")
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
