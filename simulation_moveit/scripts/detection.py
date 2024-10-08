#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import String

class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.camera_callback)
        self.cascade = cv2.CascadeClassifier('/home/jimdff/exparm_ws/src/kuka_expe_noetic/simulation_moveit/img/newcascade.xml')
        self.objectName = 'Cube'
        self.pub = rospy.Publisher("color_cube_topic", String, queue_size=10)

    def detect_color(self, roi):
        # Convert ROI to HSV color space
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Define color ranges
        lower_red = (0, 100, 100)
        upper_red = (10, 255, 255)
        lower_green = (40, 40, 40)
        upper_green = (70, 255, 255)
        lower_blue = (110, 50, 50)
        upper_blue = (130, 255, 255)
        
        # Define masks for each color
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Find the color with the maximum number of pixels
        color_masks = [(mask_red, (0, 0, 255)), (mask_green, (0, 255, 0)), (mask_blue, (255, 0, 0))]
        max_pixels = 0
        detected_color = None
        for mask, color in color_masks:
            pixels = cv2.countNonZero(mask)
            if pixels > max_pixels:
                max_pixels = pixels
                detected_color = color
        
        # If no color detected, return None
        if max_pixels == 0:
            return None
        else:
            return detected_color

    def camera_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        objects = self.cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        
        for (x, y, w, h) in objects:
            roi = cv_image[y:y+h, x:x+w]
            
            # Perform color detection on ROI
            color = self.detect_color(roi)
            
            # Skip processing if color is None
            if color is None:
                continue
            
            # Convert RGB values to color string
            color_str = None
            if color == (0, 0, 255):
                color_str = "Red"
            elif color == (0, 255, 0):
                color_str = "Green"
            elif color == (255, 0, 0):
                color_str = "Blue"
            
            # Draw rectangle and label
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), color, 2)
            cv2.putText(cv_image, self.objectName, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            
            # Publish color to topic
            if color_str is not None:
                self.pub.publish(color_str)
                
                # Print the detected color
                rospy.loginfo("Detected color: {}".format(color_str))
        
        # Display the result (you can also publish it to another topic if needed)
        cv2.imshow("Result", cv_image)
        cv2.waitKey(1)

def main():
    rospy.init_node('color_detection_node', anonymous=True)
    detector = ColorDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
