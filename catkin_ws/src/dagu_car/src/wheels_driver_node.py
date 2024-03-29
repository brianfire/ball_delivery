#!/usr/bin/env python
import rospy
from mecanum_wheels_msgs.msg import WheelsCmdStamped, BoolStamped
from dagu_wheels_driver import DaguWheelsDriver

class WheelsDriverNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.estop=False

        # Setup publishers
        self.driver = DaguWheelsDriver()
        #add publisher for wheels command wih execution time
        self.msg_wheels_cmd = WheelsCmdStamped()
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd_executed",WheelsCmdStamped, queue_size=1)

        # Setup subscribers
        self.control_constant = 1.0
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbWheelsCmd(self,msg):
        if self.estop:
            self.driver.setWheelsSpeed(left_front=0.0,right_front=0.0,left_rear=0.0,right_rear=0.0,theta=0.0)
            return
        self.driver.setWheelsSpeed(left_front=msg.vel_left_front,right_front=msg.vel_right_front,left_rear=msg.vel_left_rear,right_rear=msg.vel_right_rear,theta=msg.theta)
        #print ("send command") 
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = rospy.get_rostime()
  
        self.msg_wheels_cmd.vel_left_front = msg.vel_left_front
        self.msg_wheels_cmd.vel_right_front = msg.vel_right_front
        self.msg_wheels_cmd.vel_left_rear = msg.vel_left_rear
        self.msg_wheels_cmd.vel_right_rear = msg.vel_right_rear

        self.msg_wheels_cmd.theta = msg.theta
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)

    def cbEStop(self,msg):
        self.estop=not self.estop
        if self.estop:
            rospy.loginfo("[%s] Emergency Stop Activated")
        else:
            rospy.loginfo("[%s] Emergency Stop Released")

    def on_shutdown(self):
        self.driver.setWheelsSpeed(left_front=0.0,right_front=0.0,left_rear=0.0,right_rear=0.0,theta=0.0)
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_driver_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsDriverNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
