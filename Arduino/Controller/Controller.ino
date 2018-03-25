#include <Servo.h>
#include "data_structure.h"

// Volatile variables
volatile bool emergency_state = false; 

// Stream buffer
String read_string;

// Servomotors of the robot
Servo servo_1;
Servo servo_2;
//Servo servo_3;
Servo gripper;

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

void setup() 
{
  // Initialize the serial link
  Serial.begin(9600);

  // Interrupt routine
  attachInterrupt(0, emergency_button, CHANGE);

  // Attach the servos and go to home pos
  servo_1.attach(3, 1000, 2000);
  servo_2.attach(5, 1000, 2000);
  //servo_3.attach(7, 1000, 2000);
  gripper.attach(8);
  
  servo_1.write(home_pos.angle_1);
  servo_2.write(home_pos.angle_2);
  //servo_3.write(home_pos.angle_3);
  gripper.write(robot.gripper_state);

  // Wait 3s
  delay(3000);
}

void loop() 
{

  // Check for communication
  while (Serial.available())
  {
    // read the incoming byte:
    char c = Serial.read();
    read_string += c;
    delay(2);     
  }

  // Movin instructions
  if(read_string.length() > 0)
  {
    //convert readString into a number
    int n = read_string.toInt();

    // Check its value
    if (n <= 180 || n >= 0)
    {
      if (!emergency_state)
      {
        Serial.print("writing Angle: ");
        Serial.println(n);
        servo_1.write(n);
        servo_2.write(n);
        if (n > 90) robot.gripper_state = OPEN;
        else robot.gripper_state = CLOSE;
        gripper.write(robot.gripper_state);
        
      }

      read_string ="";
      
    }
  }
  
  

}
