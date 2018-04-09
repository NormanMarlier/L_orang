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

// pins
const byte interrupt_pin = 2;
const byte servo_0_pin = 3;
const byte servo_1_pin = 5;
const byte servo_2_pin = 7;
const byte servo_3_pin = 8;

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
Position home_pos = {0, 90, 0, CLOSE};

// Robot structure
Robot robot = {home_pos, CLOSE};

/* 
 *  Interrupt routine    
 */
void emergency_button()
{
  emergency_state = !emergency_state;
}

void cmd_motors(const std_msgs::Int32MultiArray& cmd_msg)
{
  // Change the state of the motors
  robot.pos.angle_1 = cmd_msg.data[0];
  robot.pos.angle_2 = cmd_msg.data[1];
  robot.pos.angle_3 = cmd_msg.data[2];
  if (cmd_msg.data[3] == 0)
  {
    robot.pos.angle_4 = cmd_msg.data[3];
    robot.gripper_state = CLOSE;
  }
  else if (cmd_msg.data[3] == 180)
 {
   robot.pos.angle_4 = cmd_msg.data[3];
   robot.gripper_state = OPEN;
 }
  else
  {
    robot.pos.angle_4 = cmd_msg.data[3];
    robot.gripper_state = GRABBING;
  }
}

// Ros subscriber
ros::Subscriber<std_msgs::Int32MultiArray> sub("lorang", &cmd_motors);

void setup() 
{shut
  // Interrupt routine
  attachInterrupt(0, emergency_button, RISING);

  // Init ROS
  nh.initNode();
  nh.subscribe(sub);
  
  // Attach the servos and go to home pos
  servomotors[0].attach(servo_0_pin, 1000, 2000);
  servomotors[1].attach(servo_1_pin, 1000, 2000);
  servomotors[2].attach(servo_2_pin, 1000, 2000);
  servomotors[3].attach(servo_3_pin);
  
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
    servomotors[0].write(robot.pos.angle_1);
    servomotors[1].write(robot.pos.angle_2);
    servomotors[2].write(robot.pos.angle_3);
    servomotors[3].write(robot.pos.angle_4);
  }
  
  // Callback for ROS
  nh.spinOnce();
  
  // Wait
  delay(1);

}
