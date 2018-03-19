#include <Servo.h>
#include "data_structure.h"


// Servomotors of the robot
Servo servo_1;
Servo servo_2;
Servo servo_3;

// Angle variable
float angle = 0;

// Home position
Position home_pos = {0, 90, 0};

// Robot structure
Robot robot = {home_pos, CLOSE};

void setup() 
{
  // Initialize the serial link
  Serial.begin(9600);

  // Attach the servos and go to home pos
  servo_1.attach(2, 1000, 2000);
  servo_2.attach(4, 1000, 2000);
  servo_3.attach(6, 1000, 2000);
  
  servo_1.write(home_pos.angle_1);
  servo_2.write(home_pos.angle_2);
  servo_3.write(home_pos.angle_3);

  // Wait 3s
  delay(3000);
}

void loop() 
{

  if (angle == 180) angle = 0;
  else angle += 20;
  Serial.print(angle);
  Serial.println("\n");
  robot.pos.angle_1 = angle;
  robot.pos.angle_2 = angle;
  robot.pos.angle_3 = angle;
  servo_1.write(robot.pos.angle_1);
  servo_2.write(robot.pos.angle_2);
  servo_3.write(robot.pos.angle_3);
  delay(500);
  

}
