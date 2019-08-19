#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty

if __name__ == "__main__":
    rospy.init_node("drop_by_dist_node")
    rospy.wait_for_service('/subt_info', 
    rospy.Subscriber("", 

