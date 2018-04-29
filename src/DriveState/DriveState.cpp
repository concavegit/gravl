/******************************************************************************
 * DriveState
 * @file DriveState.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * This takes in messages from teledrive and autodrive and publishes them to
 * drive based on the value published to auto (default-false)
 * auto = true:  autodrive -> drive
 * auto = false: teledrive -> drive
 *
 ******************************************************************************/


#include "DriveState.h"

/*
 * DriveState constructor
 */
DriveState::DriveState(){
  state =  n.subscribe("auto", 10, &DriveState::stateCB, this);
  teledrivesub = n.subscribe("teledrive", 10, &DriveState::teledriveCB, this);
  telehitchsub = n.subscribe("telehitch", 10, &DriveState::telehitchCB, this);
  drivepub = n.advertise<ackermann_msgs::AckermannDrive>("drive", 1000);
  hitchpub = n.advertise<geometry_msgs::Point>("hitch", 1000);
}

/*
 * Callback function for auto subscriber
 */
void DriveState::stateCB(const std_msgs::Bool &msg){
  if(msg.data){
    teledrivesub.shutdown();
    telehitchsub.shutdown();
    autodrivesub = n.subscribe("autodrive", 10, &DriveState::autodriveCB, this);
    autohitchsub = n.subscribe("autohitch", 10, &DriveState::autohitchCB, this);
  }
  else{
    autodrivesub.shutdown();
    autohitchsub.shutdown();
    teledrivesub = n.subscribe("teledrive", 10, &DriveState::teledriveCB, this);
    telehitchsub = n.subscribe("telehitch", 10, &DriveState::telehitchCB, this);
  }
}

/*
 * Callback function for teledrive subscriber - publishes to drivepub
 */
void DriveState::teledriveCB(const ackermann_msgs::AckermannDrive &msg){
  drivepub.publish(msg);
}
void DriveState::telehitchCB(const geometry_msgs::Point &msg){
  hitchpub.publish(msg);
}

/*
 * Callback function for autodrive subscriber - publishes to drivepub
 */
void DriveState::autodriveCB(const ackermann_msgs::AckermannDrive &msg){
  drivepub.publish(msg);
}
void DriveState::autohitchCB(const geometry_msgs::Point &msg){
  hitchpub.publish(msg);
}

// main function
int main(int argc, char **argv)
{
	ros::init(argc, argv, "state");
  DriveState ds;
  ros::spin();
}
