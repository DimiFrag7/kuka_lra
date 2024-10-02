#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import threading

class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.camera_callback)
        self.pose_sub = rospy.Subscriber("/arm_pose", Pose, self.pose_callback)
        self.cascade = cv2.CascadeClassifier('/home/jimdff/exparm_ws/src/kuka_expe_noetic/simulation_moveit/img/finalcascade.xml')
        self.objectName = 'Cube'
        self.pub = rospy.Publisher("color_cube_topic", String, queue_size=10)
        self.color_detected = False
        self.last_detection_time = rospy.Time.now()  # Last time a color was published
        self.last_color_str = None  # Track the last detected color
        self.arm_in_detect_pose = False  # Track if arm is in the 'detect' pose

    def detect_color(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_red = (0, 100, 100)
        upper_red = (10, 255, 255)
        lower_green = (40, 40, 40)
        upper_green = (70, 255, 255)
        lower_blue = (110, 50, 50)
        upper_blue = (130, 255, 255)

        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        color_masks = [(mask_red, (0, 0, 255)), (mask_green, (0, 255, 0)), (mask_blue, (255, 0, 0))]
        max_pixels = 0
        detected_color = None
        for mask, color in color_masks:
            pixels = cv2.countNonZero(mask)
            if pixels > max_pixels:
                max_pixels = pixels
                detected_color = color

        return detected_color

    def pose_callback(self, pose_data):
        detect_pose = Pose()
        detect_pose.position.x = 0.3
        detect_pose.position.y = 0.0
        detect_pose.position.z = 1.06
        detect_pose.orientation.w = 1.0
        
        if (abs(pose_data.position.x - detect_pose.position.x) < 0.01 and
            abs(pose_data.position.y - detect_pose.position.y) < 0.01 and
            abs(pose_data.position.z - detect_pose.position.z) < 0.01):
            self.arm_in_detect_pose = True
        else:
            self.arm_in_detect_pose = False

    def camera_callback(self, data):
        current_time = rospy.Time.now()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        objects = self.cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

        for (x, y, w, h) in objects:
            roi = cv_image[y:y+h, x:x+w]

            color = self.detect_color(roi)
            if color is None:
                continue

            color_str = None
            if color == (0, 0, 255):
                color_str = "Red"
            elif color == (0, 255, 0):
                color_str = "Green"
            elif color == (255, 0, 0):
                color_str = "Blue"

            cv2.rectangle(cv_image, (x, y), (x+w, y+h), color, 2)
            cv2.putText(cv_image, self.objectName, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

            # Only publish if 8 seconds have passed and arm is in detect pose
            if self.arm_in_detect_pose and color_str is not None:
                if (current_time - self.last_detection_time).to_sec() >= 8.0:
                    self.pub.publish(color_str)
                    rospy.loginfo("Detected color: {}".format(color_str))

                    self.last_detection_time = rospy.Time.now()  # Update last detection time

            # Continue processing image without delay for publishing
            break

        cv2.imshow("Result", cv_image)
        cv2.waitKey(1)

def main():
    rospy.init_node('color_detection_node', anonymous=True)
    detector = ColorDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
