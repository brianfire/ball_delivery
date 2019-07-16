/***************************************
Author: Wolf Chen
Date: 2019/07/03
Last update: 2019/07/03

Relay Controller
Subscribe
  /motor_on
Publish
  /relay_command
***************************************/
#include "ros/ros.h"
#include "std_msgs/Bool.h"

class relay{
private:
  std_msgs::Bool pub_msg;

  ros::NodeHandle nh;
  ros::Subscriber sub_relay;
  ros::Publisher pub_relay;

public:
  relay(ros::NodeHandle&);
  void cbRelay(const std_msgs::Bool::ConstPtr&);
};

relay::relay(ros::NodeHandle &n):
             nh(n){
  sub_relay = nh.subscribe("motor_on", 1, &relay::cbRelay, this);
  pub_relay = nh.advertise<std_msgs::Bool>("relay_command", 1);
}

void relay::cbRelay(const std_msgs::Bool::ConstPtr& sub_msg){
  if(sub_msg->data == true){
    pub_msg.data = true;

    ROS_INFO("Motor On");
    pub_relay.publish(pub_msg);
  }
  else{
    pub_msg.data = false;

    ROS_INFO("Motor Off");
    pub_relay.publish(pub_msg);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "relay_controller");
  ros::NodeHandle nh;
  ROS_INFO("Subscribing to /motor_on");
  ROS_INFO("Publishing to /relay_command");
  relay rl(nh);
  ros::spin();
  return 0;
}
