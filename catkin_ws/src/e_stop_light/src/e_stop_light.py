#!/usr/bin/env python
from __future__ import print_function

import sys
import time
import rospy
from relay_lib_seeed import *

class e_stop_light(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        print ("relay test")
        # turn all of the relays on
        relay_all_on()
        # wait a second
        time.sleep(1)
        # turn all of the relays off
        relay_all_off()
        # wait a second
        time.sleep(1)

        # Publications
        self.pub_joy_ = rospy.Publisher("~joy", Joy, queue_size=1)
        # Subscriptions
        self.sub_drop_ = rospy.Subscriber("~drop", Bool, self.cbDrop, queue_size=1)


    def cbDrop(self, drop_msg):
        if(drop_msg.data == True):
            for i in range(1, 5):
                relay_on(i)
                time.sleep(1)
                relay_off(i)




if __name__ == "__main__":
    rospy.init_node("e_stop_light",anonymous=False)
    e_stop_light = e_stop_light()
    rospy.spin()
