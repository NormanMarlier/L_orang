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
#include <std_msgs/Float32MultiArray.h>
#include "data_structure.h"

// Function declaration
void set_angle(float angle, byte motor_id);

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

void cmd_motors(const std_msgs::Float32MultiArray& cmd_msg)
{
  // Change the state of the motors
  robot.pos.angle_1 = set_angle(cmd_msg.data[0], 1);
  robot.pos.angle_2 = set_angle(cmd_msg.data[1], 2);
  robot.pos.angle_3 = set_angle(cmd_msg.data[2], 3);
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
ros::Subscriber<std_msgs::Float32MultiArray> sub("lorang", &cmd_motors);

void setup() 
{
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

// Hardware check
void set_angle(float angle, byte motor_id)
{
  if (motor_id == 1)
  {
    if (angle < ANGLE_1_MIN) robot.pos.angle_1 = ANGLE_1_MIN;
    else if (angle > ANGLE_1_MAX) robot.pos.angle_1 = ANGLE_1_MAX;
    else robot.pos.angle_1 = angle;
  }
  if (motor_id == 2)
  {
    if (angle < ANGLE_2_MIN) robot.pos.angle_2 = ANGLE_2_MIN;
    else if (angle > ANGLE_2_MAX) robot.pos.angle_2 = ANGLE_2_MAX;
    else robot.pos.angle_2 = angle;
  }
  if (motor_id == 3)
  {
    if (angle < ANGLE_3_MIN) robot.pos.angle_3 = ANGLE_3_MIN;
    else if (angle > ANGLE_3_MAX) robot.pos.angle_3 = ANGLE_3_MAX;
    else robot.pos.angle_3 = angle;
  }
}
