#ifndef DRIVE_STATE_H
#define DRIVE_STATE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Point.h>


class DriveState{
public:
  explicit DriveState();
private:
  ros::NodeHandle n;
  ros::Subscriber state;
  ros::Subscriber teledrivesub;
  ros::Subscriber autodrivesub;
  ros::Subscriber telehitchsub;
  ros::Subscriber autohitchsub;
  ros::Publisher drivepub;
  ros::Publisher hitchpub;
  ackermann_msgs::AckermannDrive drive_msg;
  geometry_msgs::Point hitch_msg
  void stateCB(const std_msgs::Bool &msg);
  void teledriveCB(const ackermann_msgs::AckermannDrive &msg);
  void autodriveCB(const ackermann_msgs::AckermannDrive &msg);
  void telehitchCB(const geometry_msgs::Point &msg);
  void autohitchCB(const geometry_msgs::Point &msg);
};

#endif //DRIVE_STATE_H
