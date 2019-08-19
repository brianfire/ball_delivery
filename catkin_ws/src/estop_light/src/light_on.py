#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from relay_lib_seeed import *

def estop_cb(msg):

	state = msg.data

	if state:
		relay_on(4)
		rospy.loginfo('Estop On')
	else:
		relay_off(4)
		rospy.loginfo('Estop Off')

if __name__ == '__main__':
	rospy.init_node('estop_light',anonymous=True)
	
	relay_off(4)
	sub_estop = rospy.Subscriber("/e_stop", Bool, estop_cb, queue_size=1)
	rospy.spin()
