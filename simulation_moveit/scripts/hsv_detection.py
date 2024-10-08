#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import String
import threading

class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.camera_callback)
        self.cascade = cv2.CascadeClassifier('/home/jimdff/exparm_ws/src/kuka_expe_noetic/simulation_moveit/img/detect_cascade.xml')
        self.objectName = 'Cube'
        self.pub = rospy.Publisher("color_cube_topic", String, queue_size=10)
        self.color_detected = False
        self.last_detection_time = rospy.Time.now()  # Initialize last detection time
        self.last_color_str = None  # Track the last detected color

    def detect_color(self, roi):
        # Convert ROI to HSV color space
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
        
        if max_pixels == 0:
            return None
        else:
            return detected_color

    def reset_color_detected(self):
        rospy.sleep(0.01) 
        self.color_detected = False

    def camera_callback(self, data):
        current_time = rospy.Time.now()
        
        # Check if 0.1 seconds have passed since the last detection
        if (current_time - self.last_detection_time).to_sec() < 0.1:
            return 
        
        if self.color_detected:
            return 
        
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
            
            # Only publish if the detected color is different from the last published color
            if color_str is not None and color_str != self.last_color_str:
                self.pub.publish(color_str)
                rospy.loginfo("Detected color: {}".format(color_str))
                
                self.color_detected = True
                self.last_detection_time = current_time  # Update the last detection time
                self.last_color_str = color_str  # Update the last detected color

                # Reset the flag after a delay in a separate thread
                threading.Thread(target=self.reset_color_detected).start()
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
