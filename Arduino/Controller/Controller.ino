/* ======================================================== *\
 * author : Norman Marlier
 * license : MIT
 * maintainer : Norman Marlier
 * email : norman.marlier@gmail.com
 * status : test
 * 
 * This code is to test ROS connection throw
 * the Rasberry Pi and the Arduino board.
\* ======================================================== */ 


// Packages
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include "data_structure.h"


// ros node
ros::NodeHandle nh;

// Volatile variables
volatile bool emergency_state = false; 

// Servomotors of the robot
Servo servomotors[4];
// servo[0] : moves the base
// servo[1] : moves the main arm
// servo[2] : moves the upper arm
// servo[3] : moves the gripper

// Angle variable
float angle = 0;

// Home position
Position home_pos = {0, 90, 0};

// Robot structure
Robot robot = {home_pos, CLOSE};

/* 
 *  Interrupt routine    
 *
 */
void emergency_button()
{
  emergency_state = !emergency_state;
}

void cmd_motors(const std_msgs::Int32MultiArray& cmd_msg)
{
  // Change the state of the motors
  robot.pos.angle1 = cmd_msg.data[0];
  robot.pos.angle2 = cmd_msg.data[1];
  robot.pos.angle3 = cmd_msg.data[2];
  if (cmd_msgs.data[3] == 0) robot.gripper_state = CLOSE;
  else if (cmd_msgs.data[3] == 180) robot.gripper_state = CLOSE;
  else robot.gripper_state = cmd_msgs.data[3];
}


// Ros subscriber
ros::Subscriber<std_msgs::Empty> sub("lorang", &cmd_motors);

void setup() 
{
  // Interrupt routine
  attachInterrupt(0, emergency_button, CHANGE);

  // Init ROS
  nh.initNode();
  nh.subscribe(sub);
  
  // Attach the servos and go to home pos
  servomotors[0].attach(3, 1000, 2000);
  servomotors[1].attach(5, 1000, 2000);
  servomotors[2].attach(7, 1000, 2000);
  servomotors[3].attach(8);
  
  servomotors[0].write(home_pos.angle_1);
  servomotors[1].write(home_pos.angle_2);
  servomotors[2].write(home_pos.angle_3);
  servomotors[3].write(robot.gripper_state);

  // Wait 3s
  delay(3000);
}

void loop() 
{
  // Update the motors state
  if (!emergency_state)
  {
    servomotors[0].write(robot.pos.angle1);
    servomotors[1].write(robot.pos.angle2);
    servomotors[2].write(robot.pos.angle3);
    servomotors[3].write(robot.gripper_state);
  }
  
  // Callback for ROS
  nh.spinOnce();
  
  // Wait
  delay(1);

}
