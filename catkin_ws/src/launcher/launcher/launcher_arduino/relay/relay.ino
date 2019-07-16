/*********************************
Author: Wolf Chen
Date: 2019/07/09
Last update: 2019/07/09

Relay Arduino Control
Subscribe:
  /relay_command
Publish:
  /relay_feedback
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

bool relayOn = true;
int relayPin = 2;

ros::NodeHandle n;

std_msgs::Bool relayFb;
ros::Publisher state("relay_feedback", &relayFb);

void cb(const std_msgs::Bool& msg){
  if(msg.data == true){
    relayOn = true;
  }
  else{
    relayOn = false;
  }
}

ros::Subscriber<std_msgs::Bool> msg_sub("/relay_command", cb);

void setup(){
  Serial.begin(57600);

  n.initNode();
  n.subscribe(msg_sub);
  n.advertise(state);

  pinMode(relayPin, OUTPUT);
}

void loop(){
  if(relayOn == true){
    digitalWrite(relayPin, HIGH);
  }
  else if(relayOn == false){
    digitalWrite(relayPin, LOW);
  }
  relayFb.data = (relayOn == true);
  state.publish(&relayFb);

  n.spinOnce();
  delay(50);
}
