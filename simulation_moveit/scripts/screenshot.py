#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ScreenshotTaker:
    def __init__(self):
        rospy.init_node('screenshot_taker', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.image_callback)
        self.screenshot_dir = os.path.expanduser('~/exparm_ws/src/kuka_expe_noetic/simulation_moveit/img/n')
        self.timer = rospy.Timer(rospy.Duration(0.25), self.take_screenshot)

    def image_callback(self, data):
        pass

    def take_screenshot(self, event):
        try:
            data = rospy.wait_for_message('image_raw', Image, timeout=1.0)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        file_name = self.screenshot_dir + '/screenshot_{}.png'.format(rospy.Time.now().to_nsec())

        try:
            cv2.imwrite(file_name, cv_image)
            rospy.loginfo("Screenshot saved as {}".format(file_name))
        except Exception as e:
            rospy.logerr("Failed to save screenshot: {}".format(e))

if __name__ == '__main__':
    try:
        screenshot_taker = ScreenshotTaker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
