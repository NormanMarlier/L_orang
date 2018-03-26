#include <Servo.h>
#include "data_structure.h"

// Volatile variables
volatile bool emergency_state = false; 

// Stream buffer
String read_string;

// Servomotors of the robot
Servo servomotors[4];
// servo[0] : moves the main arm
// servo[1] : moves the upper arm
// servo[2] : moves the base
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

void setup() 
{
  // Initialize the serial link
  Serial.begin(9600);

  // Interrupt routine
  attachInterrupt(0, emergency_button, CHANGE);

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
        servomotors[0].write(n);
        servomotors[1].write(n);
        if (n > 90) robot.gripper_state = OPEN;
        else robot.gripper_state = CLOSE;
        servomotors[3].write(robot.gripper_state);
        
      }

      read_string ="";
      
    }
  }
  
  

}
