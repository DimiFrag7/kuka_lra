#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from moveit_commander import MoveGroupCommander
from gazebo_conveyor.srv import ConveyorBeltControl  # Assuming correct package and service message
from termcolor import colored

class ArmController:
    def __init__(self):
        rospy.init_node('move_arm_position_node', anonymous=True)
        
        self.move_group = MoveGroupCommander("arm_manipulator")
        
        rospy.Subscriber("color_cube_topic", String, self.color_callback)
        
        rospy.wait_for_service('conveyor/control')
        self.conveyor_control = rospy.ServiceProxy('conveyor/control', ConveyorBeltControl)

    def color_callback(self, color_msg):
        color = color_msg.data
        rospy.loginfo("Received color: %s", color)

        if color == "Red":
            self.move_arm_to_pose("stand")
            print(f"Arm moved to 'stand' position. Color: {color}", 'red')
            self.control_conveyor_power(0, duration=10)  
            self.control_conveyor_power(10) 
        elif color == "Green":
            self.move_arm_to_pose("home")
            print(f"Arm moved to 'home' position. Color: {color}", 'green')
            self.control_conveyor_power(10) 
        elif color == "Blue":
            self.move_arm_to_pose("pick")
            print(f"Arm moved to 'stand' position. Color: {color}", 'blue')
            self.control_conveyor_power(0, duration=10)  
            self.control_conveyor_power(10) 
        else:
            rospy.logwarn("Unknown color received: %s", color)

    def move_arm_to_pose(self, position_name):
        self.move_group.set_named_target(position_name)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def control_conveyor_power(self, power, duration=None):
        try:
            response = self.conveyor_control(power)
            if response.success:
                rospy.loginfo("Conveyor power set to: %s", power)
                if duration:
                    rospy.sleep(duration)  
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

def main():
    arm_controller = ArmController()
    
    rospy.spin()

if __name__ == '__main__':
    main()
