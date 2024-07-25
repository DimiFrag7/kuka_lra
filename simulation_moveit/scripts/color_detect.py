#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CubeDetector:
    def __init__(self):
        rospy.init_node('cube_detector_node', anonymous=True)
        self.bridge = CvBridge()
        self.cube_cascade = cv2.CascadeClassifier('/home/jimdff/exparm_ws/src/kuka_expe_noetic/simulation_moveit/img/cascade.xml')

        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        frame = self.detect_and_classify_cubes(frame)

        cv2.imshow("Cube Detection", frame)
        cv2.waitKey(1)

    def detect_and_classify_cubes(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cubes = self.cube_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

        for (x, y, w, h) in cubes:
            cube_roi = frame[y:y+h, x:x+w]
            hsv_cube = cv2.cvtColor(cube_roi, cv2.COLOR_BGR2HSV)

            lower_red = np.array([0, 100, 100])
            upper_red = np.array([20, 255, 255])
            lower_blue = np.array([110, 50, 50])
            upper_blue = np.array([130, 255, 255])
            lower_green = np.array([50, 100, 100])
            upper_green = np.array([70, 255, 255])

            mask_red = cv2.inRange(hsv_cube, lower_red, upper_red)
            mask_blue = cv2.inRange(hsv_cube, lower_blue, upper_blue)
            mask_green = cv2.inRange(hsv_cube, lower_green, upper_green)

            count_red = cv2.countNonZero(mask_red)
            count_blue = cv2.countNonZero(mask_blue)
            count_green = cv2.countNonZero(mask_green)

            if count_red > count_blue and count_red > count_green:
                color = "Red"
            elif count_blue > count_red and count_blue > count_green:
                color = "Blue"
            else:
                color = "Green"

            # Draw rectangle around detected cube
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, color, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)

        return frame

if __name__ == '__main__':
    try:
        cube_detector = CubeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

