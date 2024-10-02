#!/usr/bin/env python3

import rospy
from gazebo_conveyor.srv import ConveyorBeltControl  

def conveyor_power_client(power):
    rospy.wait_for_service('conveyor/control')  
    try:
        conveyor_power = rospy.ServiceProxy('conveyor/control', ConveyorBeltControl)
        response = conveyor_power(power)
        return response.success
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    rospy.init_node('conveyor_power_client')
    power = float(input("Enter the power value: "))
    result = conveyor_power_client(power)
    print("Result:", result)
