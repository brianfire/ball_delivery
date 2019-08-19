#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from math import sqrt, atan2, pi

from __builtin__ import True

class auto_drop(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.drop_index = 1
        #self.header = None

        # Setup Parameters
        self.front_lock_time = self.setupParam("~front_lock_time", 1.1)
        self.back_lock_time = self.setupParam("~back_lock_time", 0.1)

        # Publications
        self.pub_joy_ = rospy.Publisher("~joy", Joy, queue_size=1)
        # Subscriptions
        self.sub_drop_ = rospy.Subscriber("~drop", Bool, self.cbDrop, queue_size=1)

        self.drop_srv = rospy.Service("/drop_req", SetBool, self.cbDrop)


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value


    def cbDrop(self, drop_msg):
        self.back_lock_time = 0.1
        self.front_lock_time = 1.15
        if(drop_msg.data == True):
            print "drop right ball"
            self.motor_control(3, 1, self.front_lock_time)
            self.motor_control(1, 1, self.back_lock_time)
            self.motor_control(4, 1, self.back_lock_time)
        else:
            print "drop left ball"
            self.motor_control(7, 1, self.front_lock_time)
            self.motor_control(5, 1, self.back_lock_time)
            self.motor_control(6, 1, self.back_lock_time)
 
        return True, "dropped"

    def motor_control(self, button_index, direction, duration):
        joy_msg = Joy()
        #joy_msg.header = self.header
        joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, -0.3369131088256836, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        if(button_index > 0 and button_index < 11):
            joy_msg.buttons[button_index-1] = direction
        self.pub_joy_.publish(joy_msg)
        rospy.sleep(duration)
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.pub_joy_.publish(joy_msg)
        rospy.sleep(2)

if __name__ == "__main__":
    rospy.init_node("auto_drop",anonymous=False)
    auto_drop = auto_drop()
    rospy.spin()
