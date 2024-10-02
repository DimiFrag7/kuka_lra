#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty

def call_service(kuka_grasping_on):
    rospy.wait_for_service(kuka_grasping_on)
    try:
        service_proxy = rospy.ServiceProxy(kuka_grasping_on, Empty)
        service_proxy()
        rospy.loginfo(f"Successfully called {kuka_grasping_on}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    rospy.init_node('call_all_on_services')

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
        call_service(service)

if __name__ == "__main__":
    main()
