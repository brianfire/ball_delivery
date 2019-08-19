#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, SetBool

def timer_start_cb(msg):
    drop_req = rospy.ServiceProxy("/drop_req", SetBool)
    rospy.sleep(30)
    for i in range(4):
        print "drop ball", i+1
        if (i % 2 == 0):
            drop_req(True) #drop the ball @ right
        else:
            drop_req(False) #drop the ball @ left
        rospy.sleep(120)
    return



if __name__ == "__main__":
    rospy.init_node("ball_timer_node")
    rospy.wait_for_service('/drop_req')
    timer_srv = rospy.Service("/drop_timer_start", Empty, timer_start_cb)
    print "service ready"
    rospy.spin()
