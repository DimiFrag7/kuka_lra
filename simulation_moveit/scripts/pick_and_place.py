#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from moveit_commander import MoveGroupCommander
from gazebo_conveyor.srv import ConveyorBeltControl  
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math
from std_srvs.srv import Empty

class ArmController:
    def __init__(self):
        rospy.init_node('move_arm_position_node', anonymous=True)
        
        self.move_group = MoveGroupCommander("arm_manipulator")

        self.arm_pose_pub = rospy.Publisher('/arm_pose', Pose, queue_size=10)

        self.publish_rate = rospy.Rate(5)  
        rospy.Timer(rospy.Duration(0.1), self.continuous_publish_arm_pose)  

        self.move_arm_to_pose("detect", 1, 1)
        
        rospy.Subscriber("color_cube_topic", String, self.color_callback)
        
        rospy.wait_for_service('conveyor/control') 
        self.conveyor_control = rospy.ServiceProxy('conveyor/control', ConveyorBeltControl)
        self.control_conveyor_power(15)

    def call_service(self, service_name):
        rospy.wait_for_service(service_name)
        try:
            service_proxy = rospy.ServiceProxy(service_name, Empty)
            service_proxy()
            rospy.loginfo(f"Successfully called {service_name}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def color_callback(self, color_msg):
        color = color_msg.data
        rospy.loginfo("Received color: %s", color)

        if color == "Red":
            rospy.sleep(0.5)
            self.red_movement()
        elif color == "Green":
            rospy.sleep(0.5)
            self.green_movement()
        elif color == "Blue":
            rospy.sleep(0.5)
            self.blue_movement()
        else:
            rospy.logwarn("Unknown color received: %s", color)

    def move_arm_to_pose(self, position_name, speed, acc):
        self.move_group.set_named_target(position_name)
        self.move_group.set_max_velocity_scaling_factor(speed)
        self.move_group.set_max_acceleration_scaling_factor(acc)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def publish_arm_pose(self, pose):
        self.arm_pose_pub.publish(pose)
        rospy.loginfo(f"Published arm pose: {pose}")

    def continuous_publish_arm_pose(self, event):
        current_pose = self.move_group.get_current_pose().pose
        self.publish_arm_pose(current_pose)

    def control_conveyor_power(self, power, duration=None):
        try:
            response = self.conveyor_control(power)
            if response.success:
                rospy.loginfo("Conveyor power set to: %s", power)
                if duration:
                    rospy.sleep(duration)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    
    def toggle_grasp_services(self, first_sleep, action, second_sleep):
        rospy.sleep(first_sleep)

        services = [
            f'/kuka_grasp/{action}',
            f'/kuka_grasp1/{action}',
            f'/kuka_grasp2/{action}',
            f'/kuka_grasp3/{action}',
            f'/kuka_grasp4/{action}',
            f'/kuka_grasp5/{action}',
            f'/kuka_grasp6/{action}',
            f'/kuka_grasp7/{action}',
            f'/kuka_grasp8/{action}'
        ]

        for service in services:
            self.call_service(service)

   
        rospy.sleep(second_sleep)  

    def move_to_pose(self, pose, velocity_scaling, acceleration_scaling):
        self.move_group.set_pose_target(pose)
        self.move_group.set_planning_time(20)  
        self.move_group.set_max_velocity_scaling_factor(velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(acceleration_scaling)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
       
              

    def red_movement(self):

        red_approach_pose = Pose()
        red_approach_pose.position.x = 0.3
        red_approach_pose.position.y = 0.2
        red_approach_pose.position.z = 0.8

        # Define orientation in degrees
        roll_deg = 0
        pitch_deg = 90
        yaw_deg = 0

        # Convert degrees to radians
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        # Convert RPY angles to quaternion
        quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        red_approach_pose.orientation.x = quaternion[0]
        red_approach_pose.orientation.y = quaternion[1]
        red_approach_pose.orientation.z = quaternion[2]
        red_approach_pose.orientation.w = quaternion[3]

        self.move_to_pose(red_approach_pose, 1, 1)

        self.toggle_grasp_services(0.01, 'on', 1)

        # Move the arm to the second target position
        red_lift_pose = Pose()
        red_lift_pose.position.x = 0.3
        red_lift_pose.position.y = 0.2
        red_lift_pose.position.z = 0.8 + 0.3
        red_lift_pose.orientation = red_approach_pose.orientation

        self.move_to_pose(red_lift_pose, 0.1, 0.1)

        # Rotate the arm to 130 degrees
        joint_values = self.move_group.get_current_joint_values()
        joint_values[0] = math.radians(130)
        self.move_group.set_joint_value_target(joint_values)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(0.03)
        self.move_group.set_max_acceleration_scaling_factor(0.03)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()


        # Move the arm to the red target position
        red_pose = Pose()
        red_pose.position.x = -0.42
        red_pose.position.y = -0.4
        red_pose.position.z = 0.8 + 0.3

        roll_deg = 179
        pitch_deg = 90
        yaw_deg = 0

        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        red_pose.orientation.x = quaternion[0]
        red_pose.orientation.y = quaternion[1]
        red_pose.orientation.z = quaternion[2]
        red_pose.orientation.w = quaternion[3]

        self.move_to_pose(red_pose, 0.03, 0.03)

        redplace_pose = self.move_group.get_current_pose().pose
        redplace_pose.position.z -= 0.15
        self.move_to_pose(redplace_pose, 0.03, 0.03)

        self.toggle_grasp_services(1.5, 'off', 0.01)

        self.move_arm_to_pose("detect", 1, 1)

        print(f"Arm moved to 'detect' position. Color: red")


    def green_movement(self):
        # Move the arm to the green_approach_pose
        green_approach_pose = Pose()
        green_approach_pose.position.x = 0.3
        green_approach_pose.position.y = 0.2
        green_approach_pose.position.z = 0.8

        roll_deg = 0
        pitch_deg = 90
        yaw_deg = 0

        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        green_approach_pose.orientation.x = quaternion[0]
        green_approach_pose.orientation.y = quaternion[1]
        green_approach_pose.orientation.z = quaternion[2]
        green_approach_pose.orientation.w = quaternion[3]

        self.move_to_pose(green_approach_pose, 1, 1)

        self.toggle_grasp_services(0.01, 'on', 1)

        # Move the arm to the green_lift_pose
        green_lift_pose = Pose()
        green_lift_pose.position.x = 0.3
        green_lift_pose.position.y = 0.2
        green_lift_pose.position.z = 0.8 + 0.3
        green_lift_pose.orientation = green_approach_pose.orientation

        self.move_to_pose(green_lift_pose, 0.1, 0.1)

        # Rotate the arm to 140 degrees 
        joint_values = self.move_group.get_current_joint_values()
        joint_values[0] = math.radians(140)
        self.move_group.set_joint_value_target(joint_values)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(0.03)
        self.move_group.set_max_acceleration_scaling_factor(0.03)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Move the arm to the green_pose
        green_pose  = Pose()
        green_pose.position.x = -0.42
        green_pose.position.y = -0.2
        green_pose.position.z = 0.8 + 0.3

        roll_deg = 179
        pitch_deg = 90
        yaw_deg = 0

        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        green_pose.orientation.x = quaternion[0]
        green_pose.orientation.y = quaternion[1]
        green_pose.orientation.z = quaternion[2]
        green_pose.orientation.w = quaternion[3]

        self.move_to_pose(green_pose , 0.03, 0.03)

        greenplace_pose  = self.move_group.get_current_pose().pose
        greenplace_pose .position.z -= 0.15
        self.move_to_pose(greenplace_pose , 0.03, 0.03)

        self.toggle_grasp_services(1.5, 'off', 0.01)

        self.move_arm_to_pose("detect", 1, 1)

        print(f"Arm moved to 'detect' position. Color: green")

    def blue_movement(self):
        # Move the arm to the blue_approach_pose
        blue_approach_pose = Pose()
        blue_approach_pose.position.x = 0.3
        blue_approach_pose.position.y = 0.2
        blue_approach_pose.position.z = 0.8

        roll_deg = 1
        pitch_deg = 90
        yaw_deg = 0

        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        blue_approach_pose.orientation.x = quaternion[0]
        blue_approach_pose.orientation.y = quaternion[1]
        blue_approach_pose.orientation.z = quaternion[2]
        blue_approach_pose.orientation.w = quaternion[3]

        self.move_to_pose(blue_approach_pose, 1, 1)


        self.toggle_grasp_services(0.01, 'on', 1)

        # Move the arm to the blur_lift_pose
        blur_lift_pose = Pose()
        blur_lift_pose.position.x = 0.3
        blur_lift_pose.position.y = 0.2
        blur_lift_pose.position.z = 0.8 + 0.3
        blur_lift_pose.orientation = blue_approach_pose.orientation

        self.move_to_pose(blur_lift_pose, 0.1, 0.1)


        # Rotate the arm to 120 degrees 
        joint_values = self.move_group.get_current_joint_values()
        joint_values[0] = math.radians(-120)
        self.move_group.set_joint_value_target(joint_values)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(0.03)
        self.move_group.set_max_acceleration_scaling_factor(0.03)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()


        # Move the arm to the blue_pose
        blue_pose = Pose()
        blue_pose.position.x = -0.4
        blue_pose.position.y = 0.3
        blue_pose.position.z = 0.8 + 0.3

        roll_deg = 179
        pitch_deg = 90
        yaw_deg = 0

        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        blue_pose.orientation.x = quaternion[0]
        blue_pose.orientation.y = quaternion[1]
        blue_pose.orientation.z = quaternion[2]
        blue_pose.orientation.w = quaternion[3]

        self.move_to_pose(blue_pose, 0.03, 0.03)


        # Move the arm slightly down to place the object
        blueplace_pose = self.move_group.get_current_pose().pose
        blueplace_pose.position.z -= 0.15
        self.move_to_pose(blueplace_pose, 0.03, 0.03)

        self.toggle_grasp_services(1.5, 'off', 0.01)

        self.move_arm_to_pose("detect", 1, 1)

        print(f"Arm moved to 'detect' position. Color: blue")   
                       

def main():
    
    arm_controller = ArmController()
    
    rospy.spin()

if __name__ == '__main__':
    main()
