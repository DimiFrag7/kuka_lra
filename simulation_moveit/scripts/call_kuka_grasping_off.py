#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty

def call_service(kuka_grasping_off):
    rospy.wait_for_service(kuka_grasping_off)
    try:
        service_proxy = rospy.ServiceProxy(kuka_grasping_off, Empty)
        service_proxy()
        rospy.loginfo(f"Successfully called {kuka_grasping_off}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    rospy.init_node('call_all_off_services')

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
        call_service(service)

if __name__ == "__main__":
    main()
