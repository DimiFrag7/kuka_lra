#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from moveit_commander import MoveGroupCommander
from gazebo_conveyor.srv import ConveyorBeltControl  
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates

class ArmController:
    def __init__(self):
        rospy.init_node('move_arm_position_node', anonymous=True)
        
        # Move group for controlling the arm
        self.move_group = MoveGroupCommander("arm_manipulator")

        # Publisher for arm pose
        self.arm_pose_pub = rospy.Publisher('/arm_pose', Pose, queue_size=10)

        # Start a timer to publish arm pose continuously at 10 Hz
        self.publish_rate = rospy.Rate(5)  # 10 Hz publishing rate
        rospy.Timer(rospy.Duration(0.1), self.continuous_publish_arm_pose)  # Calls every 0.1 seconds

        # Initial movement to "detect" pose
        self.move_arm_to_pose("detect", 1, 1)
        
        # Subscribe to color detection topic
        rospy.Subscriber("color_cube_topic", String, self.color_callback)
        
        # Conveyor service
        rospy.wait_for_service('conveyor/control')
        self.conveyor_control = rospy.ServiceProxy('conveyor/control', ConveyorBeltControl)
        self.control_conveyor_power(15)

    def model_states_callback(self, msg):
        try:
            cube_index = msg.name.index("cube")
            self.cube_y = msg.pose[cube_index].position.y
        except ValueError:
            rospy.logerr("Cube not found")

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
        # This method gets called continuously by the rospy.Timer to publish arm pose
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

    def red_movement(self):
        # Move the arm to the first target position
        first_pose = Pose()
        first_pose.position.x = 0.3
        first_pose.position.y = 0.2
        first_pose.position.z = 0.8

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
        first_pose.orientation.x = quaternion[0]
        first_pose.orientation.y = quaternion[1]
        first_pose.orientation.z = quaternion[2]
        first_pose.orientation.w = quaternion[3]

        self.move_group.set_pose_target(first_pose)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(1)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        rospy.sleep(0.01)

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
            self.call_service(service)

        rospy.sleep(2)

        # Move the arm to the second target position
        second_pose = Pose()
        second_pose.position.x = 0.3
        second_pose.position.y = 0.2
        second_pose.position.z = 0.8 + 0.3
        second_pose.orientation = first_pose.orientation

        self.move_group.set_pose_target(second_pose)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

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

        self.move_group.set_pose_target(red_pose)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(0.03)
        self.move_group.set_max_acceleration_scaling_factor(0.03)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        redplace_pose = self.move_group.get_current_pose().pose
        redplace_pose.position.z -= 0.15
        self.move_group.set_pose_target(redplace_pose)
        self.move_group.set_planning_time(20)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

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
            self.call_service(service)

        self.move_arm_to_pose("detect", 1, 1)

        print(f"Arm moved to 'detect' position. Color: red")


    def green_movement(self):
        # Move the arm to the first target position
        first_pose = Pose()
        first_pose.position.x = 0.3
        first_pose.position.y = 0.2
        first_pose.position.z = 0.8

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
        first_pose.orientation.x = quaternion[0]
        first_pose.orientation.y = quaternion[1]
        first_pose.orientation.z = quaternion[2]
        first_pose.orientation.w = quaternion[3]

        self.move_group.set_pose_target(first_pose)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(1)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        rospy.sleep(0.01)

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
            self.call_service(service)

        rospy.sleep(2)

        # Move the arm to the second target position
        second_pose = Pose()
        second_pose.position.x = 0.3
        second_pose.position.y = 0.2
        second_pose.position.z = 0.8 + 0.3
        second_pose.orientation = first_pose.orientation

        self.move_group.set_pose_target(second_pose)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

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

 # Move the arm to the red target position
        red_pose = Pose()
        red_pose.position.x = -0.42
        red_pose.position.y = -0.2
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

        self.move_group.set_pose_target(red_pose)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(0.03)
        self.move_group.set_max_acceleration_scaling_factor(0.03)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        redplace_pose = self.move_group.get_current_pose().pose
        redplace_pose.position.z -= 0.15
        self.move_group.set_pose_target(redplace_pose)
        self.move_group.set_planning_time(20)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

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
            self.call_service(service)

        self.move_arm_to_pose("detect", 1, 1)

        print(f"Arm moved to 'detect' position. Color: green")

    def blue_movement(self):
        # Move the arm to the first target position
        first_pose = Pose()
        first_pose.position.x = 0.3
        first_pose.position.y = 0.2
        first_pose.position.z = 0.8

        # Define orientation in degrees
        roll_deg = 1
        pitch_deg = 90
        yaw_deg = 0

        # Convert degrees to radians
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        # Convert RPY angles to quaternion
        quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        first_pose.orientation.x = quaternion[0]
        first_pose.orientation.y = quaternion[1]
        first_pose.orientation.z = quaternion[2]
        first_pose.orientation.w = quaternion[3]

        self.move_group.set_pose_target(first_pose)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(1)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        rospy.sleep(0.01)

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
            self.call_service(service)

        rospy.sleep(0.01)

        # Move the arm to the second target position
        second_pose = Pose()
        second_pose.position.x = 0.3
        second_pose.position.y = 0.2
        second_pose.position.z = 0.8 + 0.3
        second_pose.orientation = first_pose.orientation

        self.move_group.set_pose_target(second_pose)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Rotate the arm to 120 degrees (2.094 radians)
        joint_values = self.move_group.get_current_joint_values()
        joint_values[0] = math.radians(-120)
        self.move_group.set_joint_value_target(joint_values)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(0.03)
        self.move_group.set_max_acceleration_scaling_factor(0.03)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Move the arm to the blue target position
        blue_pose = Pose()
        blue_pose.position.x = -0.4
        blue_pose.position.y = 0.3
        blue_pose.position.z = 0.8 + 0.3

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
        blue_pose.orientation.x = quaternion[0]
        blue_pose.orientation.y = quaternion[1]
        blue_pose.orientation.z = quaternion[2]
        blue_pose.orientation.w = quaternion[3]

        self.move_group.set_pose_target(blue_pose)
        self.move_group.set_planning_time(20)
        self.move_group.set_max_velocity_scaling_factor(0.03)
        self.move_group.set_max_acceleration_scaling_factor(0.03)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Move the arm slightly down to place the object
        blueplace_pose = self.move_group.get_current_pose().pose
        blueplace_pose.position.z -= 0.15
        self.move_group.set_pose_target(blueplace_pose)
        self.move_group.set_planning_time(20)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

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
            self.call_service(service)

        self.move_arm_to_pose("detect", 1, 1)

        print(f"Arm moved to 'detect' position. Color: blue")   
                       

def main():
    
    arm_controller = ArmController()
    
    rospy.spin()

if __name__ == '__main__':
    main()
