#ifndef TELEOP_H
#define TELEOP_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>

class Teleop{
public:
  explicit Teleop();
private:
  ros::NodeHandle n;
  ros::Subscriber joystick;
  ros::Publisher teledrive;
  ros::Publisher telehitch;
  ros::Publisher softestop;
  ros::Publisher autonomous;
  std_msgs::Bool stop_msg;
  std_msgs::Bool autonomous_msg;
  ackermann_msgs::AckermannDrive drive_msg;
  geometry_msgs::Point hitch_msg;

  void joyCB(const sensor_msgs::Joy::ConstPtr &joy);
  void stop_pub(bool stop);
  void auto_pub(bool aut);

};

#endif //TELEOP_H
