/*********************************
Author: Wolf Chen
Date: 2019/07/02
Last update: 2019/07/03

Switcher Arduino Control
Subscribe:
  /switcher_command
Publish:
  /switcher_feedback
********************************/
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <ros.h>

bool shoot = true;
int shoot_ang = 90;
int place_ang = 0;

ros::NodeHandle n;

std_msgs::Bool switcher_feedback;
ros::Publisher state("switcher_status", &switcher_feedback);

Servo switcher;

void cb(const std_msgs::Bool& msg){
  if(msg.data == true){
    shoot = true;
  }
  else{
    shoot = false;
  }
}

ros::Subscriber<std_msgs::Bool> msg_sub("/switcher_command", cb);

void setup(){
  Serial.begin(57600);

  n.initNode();
  n.subscribe(msg_sub);
  n.advertise(state);

  switcher.attach(3);  //MG996R
}

void loop(){
  if(shoot == true){
    switcher.write(shoot_ang);
  }
  else if(shoot == false){
    switcher.write(place_ang);
  }
  switcher_feedback.data = (shoot == true);
  state.publish(&switcher_feedback);

  n.spinOnce();
  delay(500);
}
