#include <Encoder.h>
#include "ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>

Encoder WireEncoder(18, 19);
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

long positionWireEncoder  = -999;

void loop() {
  long newWireEncoder;
  float HeightOfHitch;
  float WireEncoderInInches;
  newWireEncoder = WireEncoder.read(); 
  if (newWireEncoder != positionWireEncoder) {
    WireEncoderInInches = newWireEncoder / 1000.0;
    Serial.println(WireEncoderInInches);
    HeightOfHitch = WireEncoderInInches * 1.1429 + 1.7474;
    pnt_msg.z = HeightOfHitch;
    positionWireEncoder = newWireEncoder;
  }
  hitch_pose.publish(&pnt_msg);
  nh.spinOnce();
}
