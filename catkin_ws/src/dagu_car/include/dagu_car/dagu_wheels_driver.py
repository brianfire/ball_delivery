#!/usr/bin/python

# Wrapping the Adafruit API to talk to DC motors with a simpler interface
#
# date:    11/17/2015
#
# authors: Valerio Varricchio <valerio@mit.edu>
#          Luca Carlone <lcarlone@mit.edu>
#          Dmitry Yershov <dmitry.s.yershov@gmail.com>
#          Shih-Yuan Liu <syliu@mit.edu>
#
# Modified for mecanum platform usage
#
# date:    10/29/1018
#
# author:  Allen Ou <bicycleboy323@gmail.com>
import rospy
from Adafruit_MotorHAT import Adafruit_MotorHAT
from math import fabs, floor, sin, cos, pi

class DaguWheelsDriver:
    LEFT_MOTOR_MIN_PWM = 60        # Minimum speed for left motor  
    LEFT_MOTOR_MAX_PWM = 255       # Maximum speed for left motor  
    RIGHT_MOTOR_MIN_PWM = 60       # Minimum speed for right motor  
    RIGHT_MOTOR_MAX_PWM = 255      # Maximum speed for right motor  
    # AXEL_TO_RADIUS_RATIO = 1.0     # The axel length and turning radius ratio
    SPEED_TOLERANCE = 1.e-2       # speed tolerance level

    def __init__(self, verbose=False, debug=False, left_flip=False, right_flip=False):
        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.leftFrontMotor = self.motorhat.getMotor(1)
        self.rightFrontMotor = self.motorhat.getMotor(2)
	#
	self.leftRearMotor = self.motorhat.getMotor(3)
	self.rightRearMotor = self.motorhat.getMotor(4)
	#
        self.verbose = verbose or debug
        self.debug = debug
        
        self.left_sgn = 1.0
        if left_flip:
            self.left_sgn = -1.0

        self.right_sgn = 1.0
        if right_flip:
            self.right_sgn = -1.0

        self.leftFrontSpeed = 0.0
        self.rightFrontSpeed = 0.0
        self.leftRearSpeed = 0.0
        self.rightRearSpeed = 0.0

        self.theta = 0.0
        self.updatePWM()

    def PWMvalue(self, v, minPWM, maxPWM):
        pwm = 0
        if fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(v) * (maxPWM - minPWM) + minPWM))
        return min(pwm, maxPWM)

    def updatePWM(self):
        vlf = self.leftFrontSpeed*self.left_sgn*sin(-1*self.theta+3*pi/4)
        vrf = self.rightFrontSpeed*self.right_sgn*cos(-1*self.theta+3*pi/4)
        vlr = self.leftRearSpeed*self.left_sgn*cos(-1*self.theta+3*pi/4)
        vrr = self.rightRearSpeed*self.right_sgn*sin(-1*self.theta+3*pi/4)
        #rospy.loginfo("vlf:%s vrf:%s vlr:%s vrr:%s" %(vlf,vrf,vlr,vrr))

        pwmlf = self.PWMvalue(vlf, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmrf = self.PWMvalue(vrf, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)
        pwmlr = self.PWMvalue(vlr, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmrr = self.PWMvalue(vrr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)
        #rospy.loginfo("pwmlf:%s pwmrf:%s pwmlr:%s pwmrr:%s" %(pwmlf,pwmrf,pwmlr,pwmrr))
        if self.debug:
            print "v = %5.3f, u = %5.3f, vl = %5.3f, vr = %5.3f, pwml = %3d, pwmr = %3d" % (v, u, vl, vr, pwml, pwmr)

        if fabs(vlf) < self.SPEED_TOLERANCE:
            leftFrontMotorMode = Adafruit_MotorHAT.RELEASE
            pwmlf = 0
        elif vlf > 0:
            leftFrontMotorMode = Adafruit_MotorHAT.FORWARD
        elif vlf < 0: 
            leftFrontMotorMode = Adafruit_MotorHAT.BACKWARD

        if fabs(vrf) < self.SPEED_TOLERANCE:
            rightFrontMotorMode = Adafruit_MotorHAT.RELEASE
            pwmrf = 0
        elif vrf > 0:
            rightFrontMotorMode = Adafruit_MotorHAT.FORWARD
        elif vrf < 0: 
            rightFrontMotorMode = Adafruit_MotorHAT.BACKWARD

        if fabs(vlr) < self.SPEED_TOLERANCE:
            leftRearMotorMode = Adafruit_MotorHAT.RELEASE
            pwmlr = 0
        elif vlr > 0:
            leftRearMotorMode = Adafruit_MotorHAT.FORWARD
        elif vlr < 0:
            leftRearMotorMode = Adafruit_MotorHAT.BACKWARD

        if fabs(vrr) < self.SPEED_TOLERANCE:
            rightRearMotorMode = Adafruit_MotorHAT.RELEASE
            pwmrr = 0
        elif vrr > 0:
            rightRearMotorMode = Adafruit_MotorHAT.FORWARD
        elif vrr < 0:
            rightRearMotorMode = Adafruit_MotorHAT.BACKWARD


        self.leftFrontMotor.setSpeed(pwmlf)
        self.leftFrontMotor.run(leftFrontMotorMode)
        self.rightFrontMotor.setSpeed(pwmrf)
        self.rightFrontMotor.run(rightFrontMotorMode)
        self.leftRearMotor.setSpeed(pwmlr)
	self.leftRearMotor.run(leftRearMotorMode)
        self.rightRearMotor.setSpeed(pwmrr)
        self.rightRearMotor.run(rightRearMotorMode)

    def setWheelsSpeed(self, left_front, right_front, left_rear, right_rear, theta):
        self.leftFrontSpeed = left_front
        self.rightFrontSpeed = right_front
        self.leftRearSpeed = left_rear
        self.rightRearSpeed = right_rear
        #rospy.loginfo("leftSpeed:%s rightSpeed:%s" %(left,right))
        self.theta = theta
        self.updatePWM()

    def __del__(self):
        self.leftFrontMotor.run(Adafruit_MotorHAT.RELEASE)
        self.rightFrontMotor.run(Adafruit_MotorHAT.RELEASE)
        self.leftRearMotor.run(Adafruit_MotorHAT.RELEASE)
        self.rightRearMotor.run(Adafruit_MotorHAT.RELEASE)
        del self.motorhat

# Simple example to test motors
if __name__ == '__main__':
    from time import sleep

    N = 10
    delay = 100. / 1000.

    dagu = DAGU_Differential_Drive()

    # turn left
    dagu.setSteerAngle(1.0)
    # accelerate forward
    for i in range(N):
        dagu.setSpeed((1.0 + i) / N)
        sleep(delay)
    # decelerate forward
    for i in range(N):
        dagu.setSpeed((-1.0 - i + N) / N)
        sleep(delay)

    # turn right
    dagu.setSteerAngle(-1.0)
    # accelerate backward
    for i in range(N):
        dagu.setSpeed(-(1.0 + i) / N)
        sleep(delay)
    # decelerate backward
    for i in range(N):
        dagu.setSpeed(-(-1.0 - i + N) / N)
        sleep(delay)

    # turn left
    dagu.setSteerAngle(1.0)
    # accelerate forward
    for i in range(N):
        dagu.setSpeed((1.0 + i) / N)
        sleep(delay)
    # decelerate forward
    for i in range(N):
        dagu.setSpeed((-1.0 - i + N) / N)
        sleep(delay)

    del dagu
