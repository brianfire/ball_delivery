#!/usr/bin/python
import rospy
from Adafruit_MotorHAT import Adafruit_MotorHAT
import math
import time
import atexit

class Slideway(object):
    def __init__(self, addr=0x60, left_one_id=1, left_two_id=2, right_one_id=3, right_two_id=4):
        # Setup ROS node
        self.node_name = rospy.get_name()

        # Setup motor name
        self._mh = Adafruit_MotorHAT(addr)
        self._leftOne = self.mh.getMotor(left_one_id)
        self._leftTwo = self.mh.getMotor(left_two_id)
        self._rightOne = self.mh.getMotor(right_one_id)
        self._rightTwo = self.mh.getMotor(right_two_id)

        # Start with motor turn off
        self._leftOne.run(Adafruit_MotorHAT.RELEASE)
        self._leftTwo.run(Adafruit_MotorHAT.RELEASE)
        self._rightOne.run(Adafruit_MotorHAT.RELEASE)
        self._rightTwo.run(Adafruit_MotorHAT.RELEASE)

        # Setup the publisher and subscriber
        self.sub_slideway_cmd = rospy.Subscriber("~slideway_cmd", )
