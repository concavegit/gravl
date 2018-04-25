/**********************************************************************
 * KUBO Hindbrain Code (Teensy 3.5)
 * @file hind_brain.ino
 * @author: Connor Novak
 * @email: connor.novak@students.olin.edu
 * @version: 1.3
 *
 * Basic OAK_compatible control of velocity actuator and
 * steering actuator through ackermann steering messages
 * over /drive, autonomous activation through boolean
 * message over /auto, estop capability over /softestop
 **********************************************************************/

// Include Libraries
#include "RoboClaw.h"                       // Used for motor controller interface
#include <Arduino.h>                        // Used for Arduino functions
#include "ros.h"                            // Used for rosserial communication
#include "ackermann_msgs/AckermannDrive.h"  // Used for rosserial steering message
#include "geometry_msgs/Point.h"
#include "estop.h"                          // Used to implement estop class
#include "soft_switch.h"                    // Used to implement auto switch

// Declare switch & estop
Estop *e;
OAKSoftSwitch *l;

// Def/Init Constants ----------C----------C----------C

// Physical Pins
const byte AUTO_LED_PIN = 3;
const byte ESTOP_PIN = 2;

// RoboClaw & Settings
#define RC_SERIAL Serial1
#define address 0x80
#define ROBOCLAW_UPDATE_RATE 500
RoboClaw rc1(&Serial1, 10000); // In front box, for steering and driving
RoboClaw rc2(&Serial2, 10000); // In back of tractor, for hitch

// General Constants
// RoboClaw 1
#define DEBUG TRUE
const int VEL_HIGH = 2048;
const int VEL_LOW = 190;
const int VEL_CONTROL_RANGE = 2;    // Range of incoming signals
const int STEER_HIGH = 1200;
const int STEER_LOW = 600;
const int STEER_CONTROL_RANGE = 90;
const byte VEL_FIDELITY = 10;       // Stepping sub-division of actuator
const byte STEER_FIDELITY = 1;
// RoboClaw 2
const int HEIGHT_MAX = 2048; // Retracted Actuator
const int HEIGHT_MIN = 190;  // Extended Actuator
const int HEIGHT_CONTROL_RANGE = 2;    // Range of incoming signals

// Def/Init Global Variables ----------V----------V----------V

// States
boolean isEStopped = false;
boolean isAuto = false;

// RoboClaw 1
int prevVelMsg;
unsigned int velMsg = VEL_HIGH;                     // High vel var = low vel
int prevSteerMsg;
signed int steerMsg = (STEER_HIGH + STEER_LOW) / 2; // Straight steer in middle
unsigned long prevMillis = millis();
// RoboClaw 2
int prevHeightMsg;
unsigned int heightMsg = (HEIGHT_MAX + HEIGHT_MIN) / 2; // High height_max = retracted actuator = raise hitch



/*
 * FUNCTION: ackermannCB()
 * DESC: Called upon msg receipt from /drive; saves data to global vars
 * ARGS: ros ackermanndrive message
 * RTNS: none
 */
void ackermannCB(const ackermann_msgs::AckermannDrive &drive){
  steerMsg = steerConvert(drive.steering_angle);
  velMsg = velConvert(drive.speed);
  
} //ackermannCB()

/*
 * FUNCTION: hitchCB()
 * DESC: Called upon msg receipt from /hitch; saves data to global vars
 * ARGS: ros point message
 * RTNS: none
 */
void hitchCB(const geometry_msgs::Point &hitch){
  heightMsg = heightConvert(hitch.z);
  
} //hitchCB()

// Declare ROS node & subscriber
ros::NodeHandle nh;
ros::Subscriber<ackermann_msgs::AckermannDrive> sub_drive("drive", &ackermannCB);
ros::Subscriber<geometry_msgs::Point> sub_hitch("hitch", &hitchCB);


/*
 * FUNCTION: setup()
 * DESC: runs once on startup
 * ARGS: none
 * RTNS: none
 */
void setup() { // ----------S----------S----------S----------S----------S

  //Open serial communication with roboclaw
  rc1.begin(38400);
  rc2.begin(38400);

  // Set up ROS node and initialize subscriber
  nh.getHardware()->setBaud(115200);
  nh.initNode(); // Initialize ROS nodehandle
  nh.subscribe(sub_drive);
  nh.subscribe(sub_hitch);

  // Initialize estop and auto-switch
  e = new Estop(&nh, ESTOP_PIN, 1);
  pinMode(ESTOP_PIN, OUTPUT);
  l = new OAKSoftSwitch(&nh, "/auto", AUTO_LED_PIN);

  // Provide estop and estart functions
  e->onStop(eStop);
  e->offStop(eStart);

  // TODO Operator verify that actuators are in default positions

  // Set actuators to default positions
  rc1.SpeedAccelDeccelPositionM1(address, 0, 300, 0, velMsg, 0);
  prevVelMsg = velMsg;
  rc1.SpeedAccelDeccelPositionM2(address, 0, 500, 0, steerMsg, 0);
  prevSteerMsg = steerMsg;
  rc2.SpeedAccelDeccelPositionM2(address, 0, 300, 0, heightMsg, 0);
  prevHeightMsg = heightMsg;

} //setup()


/*
 * FUNCTION: loop()
 * DESC:  loops constantly
 * ARGS: none
 * RTNS: none
*/
void loop() { // ----------L----------L----------L----------L----------L

  // Checks for connectivity with mid-brain and updates estopped state
  checkSerial(&nh);
  
  // Sends commands to RoboClaw every ROBOCLAW_UPDATE_RATE milliseconds
  if (millis() - prevMillis > ROBOCLAW_UPDATE_RATE && !isEStopped) {
    updateRoboClaw(velMsg, steerMsg, heightMsg);

  }
    
  // Updates node
  nh.spinOnce();
  delay(1);

} //loop()


// ----------F----------F----------F----------F----------F----------F----------F

/*
 * FUNCTION: checkSerial()
 * DESC: Estops if node isn't connected
 * ARGS: nodehandle to check for connectivity
 * RTNS: none
 */
 void checkSerial(ros::NodeHandle *nh) {

  // If node isn't connected and tractor isn't estopped, estop
  if(!nh->connected()) {
    if(!isEStopped) {
      eStop();
    }
  }
 } //checkSerial()


/*
 * FUNCTION: updateRoboClaw()
 * DESC: Sends current velocity and steering vals to RoboClaw; called at ROBOCLAW_UPDATE_RATE
 * ARGS: integer velocity, integer steering angle
 * RTNS: none
*/
void updateRoboClaw(int velMsg, int steerMsg, int heightMsg) {

  //Calculate step sizes based on fidelity
  int steerStep = (STEER_HIGH - STEER_LOW) / STEER_FIDELITY;
  int velStep = (VEL_HIGH - VEL_LOW) / VEL_FIDELITY;

  //Update velMsg based on step
  stepActuator(&velMsg, &prevVelMsg, velStep);
  stepActuator(&steerMsg, &prevSteerMsg, steerStep);
  
  // Update prev msgs
  prevVelMsg = velMsg;
  prevSteerMsg = steerMsg;
  prevHeightMsg = heightMsg;
  
  // Write velocity to RoboClaw
  rc1.SpeedAccelDeccelPositionM1(address, 100000, 1000, 0, velMsg, 0);

  // Write steering to RoboClaw if tractor is moving, else returns debug msg
  if (velMsg < VEL_HIGH) {
    rc1.SpeedAccelDeccelPositionM2(address, 0, 1000, 0, steerMsg, 0);
  }
  else {
    #ifdef DEBUG
    char i[48];
    snprintf(i, sizeof(i), "ERR: tractor not moving, steering message failed");
    nh.loginfo(i);
    #endif //DEBUG
  }

  // Write hitch height to RoboClaw
  rc2.SpeedAccelDeccelPositionM2(address, 100000, 1000, 0, heightMsg, 0);

  prevMillis = millis();  // Reset timer

  // roslog msgs if debugging
  #ifdef DEBUG
    char j[36];
    snprintf(j, sizeof(j), "DBG: steerMsg = %d, velMsg = %d, heightMsg = %d", steerMsg, velMsg, heightMsg);
    nh.loginfo(j);
  #endif //DEBUG

} //updateRoboClaw()


/*
 * FUNCTION: steerConvert()
 * DESC: Converts ackermann steering angle to motor encoder value for RoboClaw
 * ARGS: float ackermann steering angle
 * RTNS: converted encoder steering angle
 */
int steerConvert(float ack_steer){

  // Convert from range of input signal to range of output signal, then shift signal
  ack_steer = ack_steer * ((STEER_HIGH - STEER_LOW) / STEER_CONTROL_RANGE) + (STEER_HIGH + STEER_LOW) / 2;

  // Safety limits for signal (double safety, RoboClaw already does this)
  if (ack_steer > STEER_HIGH) {
    ack_steer = STEER_HIGH;
  }
  else if (ack_steer < STEER_LOW) {
    ack_steer = STEER_LOW;
  }

  // Switches steering dir
  ack_steer = STEER_HIGH - (ack_steer - STEER_LOW);

  return ack_steer;
} //steerConvert


/*
 * FUNCTION: velConvert()
 * DESC: Converts ackermann velocity to motor encoder value for RoboClaw
 * ARGS: float ackermann velocity
 * RTRNS: converted ackermann velocity
 */
int velConvert(float ack_vel){

  // filter to remove tractor reversal commands (platform wont back up)
  if (ack_vel < 0) {
    ack_vel = 0;
  }

  // Convert from range of input signal to range of output signal
  ack_vel = VEL_HIGH - ack_vel * ((VEL_HIGH - VEL_LOW) / VEL_CONTROL_RANGE);

  return ack_vel;
} //velConvert()



// TODO: MAKE THIS USE ENCODER DATA
/*
 * FUNCTION: heightConvert()
 * DESC: CURRENTLY CONVERTS -1 to 1 POSITION TO POSITION FOR ROBOCLAW
 * ARGS: float position
 * RTRNS: converted position
 */
int heightConvert(float hitch_pos){
  
  hitch_pos = HEIGHT_MAX - hitch_pos * ((HEIGHT_MAX - HEIGHT_MIN) / HEIGHT_CONTROL_RANGE);

  return hitch_pos;
  }



/*
 * FUNCTION: stepActuator()
 * DESC: Meters commands sent to motors to ensure quick response and low latency
 * ARGS: current motor message, previous motor message, step size to check
 * RTNS: none
 */

void stepActuator(int *msg, int *prevMsg, int step) {

  // Checks if stepping is necessary (input signal wants to increase by more than the given step size)
  if (abs(*prevMsg - *msg) > step) {

    // Logs step verification if debugging
    #ifdef DEBUG
    nh.loginfo("DBG: Stepping signal");
    #endif //DEBUG

    // If signal increasing, step up
    if (*msg > *prevMsg) {
      *msg = *prevMsg + step;
    }

    // If signal decreasing, step down
    else if (*msg < *prevMsg) {
      *msg = *prevMsg - step;
    }

    // Exception case
    else {
      *msg = *prevMsg;
    }
  }
} //stepActuator()


/*
 *  FUNCTION eStop()
 *  DESC: Estops tractor, sends error message, flips estop state
 *  ARGS: none
 *  RTRNS: none
 */
void eStop() {

  isEStopped = true;

  // Logs estop msg
  char i[32];
  snprintf(i, sizeof(i), "ERR: Tractor E-Stopped");
  nh.loginfo(i);

  // Toggle relay to stop engine
  digitalWrite(ESTOP_PIN, HIGH);
  delay(2000);
  digitalWrite(ESTOP_PIN, LOW);


} //eStop()


/*
 * FUNCTION eStart()
 * DESC: Changes estopped state upon tractor restart
 * ARGS: none
 * RTNS: none
 */
 void eStart() {
  isEStopped = false;

  // Logs verification msg
  char i[32];
  snprintf(i, sizeof(i), "MSG: EStop Disactivated");
  nh.loginfo(i);

 } //eStart()
