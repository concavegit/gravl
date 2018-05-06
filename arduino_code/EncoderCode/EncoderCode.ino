#include <Encoder.h>
#include "ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>

Encoder knobLeft(18, 19);
ros::NodeHandle nh;
geometry_msgs::Point pnt_msg;
ros::Publisher hitch_pose("hitch_pose",&pnt_msg);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
  nh.initNode();
  nh.advertise(hitch_pose);
  pnt_msg.x = 0.0;
  pnt_msg.y = 0.0;
  pnt_msg.z = 0.0;
}

long positionLeft  = -999;

void loop() {
  long newLeft;
  float HeightOfHitch;
  float LeftInInches;
  newLeft = knobLeft.read();  
  if (newLeft != positionLeft) {
    LeftInInches = newLeft / 1000.0;
    HeightOfHitch = LeftInInches * 1.1429 + 1.7474;
    pnt_msg.z = HeightOfHitch;
    positionLeft = newLeft;
  }
  hitch_pose.publish(&pnt_msg);
  nh.spinOnce();
}
