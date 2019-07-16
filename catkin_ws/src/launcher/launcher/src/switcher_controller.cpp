/***************************************
Author: Wolf Chen
Date: 2019/07/03
Last update: 2019/07/03

Switcher Controller
Subscribe
  /launcher_switch
Publish
  /switcher_command
***************************************/
#include "ros/ros.h"
#include "std_msgs/Bool.h"

class switcher{
private:
  std_msgs::Bool pub_msg;

  ros::NodeHandle nh;
  ros::Subscriber sub_switch;
  ros::Publisher pub_switch;

public:
  switcher(ros::NodeHandle&);
  void cbSwitcher(const std_msgs::Bool::ConstPtr&);
};

switcher::switcher(ros::NodeHandle &n):
                   nh(n){
  sub_switch = nh.subscribe("launcher_switch", 1, &switcher::cbSwitcher, this);
  pub_switch = nh.advertise<std_msgs::Bool>("switcher_command", 1);
}

void switcher::cbSwitcher(const std_msgs::Bool::ConstPtr& sub_msg){
  // shooting
  if(sub_msg->data == true){
    pub_msg.data = true;

    ROS_INFO("Shooting");
    pub_switch.publish(pub_msg);
  }
  // placing
  else{
    pub_msg.data = false;

    ROS_INFO("Placing");
    pub_switch.publish(pub_msg);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "switcher_controller");
  ros::NodeHandle nh;
  ROS_INFO("Subscribing to /launcher_switcher");
  ROS_INFO("Publishing to /switcher_command");
  switcher sw(nh);
  ros::spin();
  return 0;
}
