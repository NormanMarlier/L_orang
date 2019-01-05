#include <Servo.h>


// pins
const byte servo_pin_1 = 7;
const byte servo_pin_2 = 8;
const byte servo_pin_3 = 3;

// Stream buffer
String readString;

// Servo
Servo servomotor;
Servo servomotor2;
Servo servomotor3;

// Angle variable
unsigned char angle = 0;


void setup() 
{
  // Initialize the serial link
  Serial.begin(9600);

  // Attahc the servo
  servomotor.attach(servo_pin_3);
  servomotor2.attach(servo_pin_2);
  servomotor3.attach(servo_pin_1);
}

void loop() 
{
  // Get the data
  while (Serial.available()) 
  {
    // read the incoming byte:
    char c = Serial.read();
    readString += c;
    delay(2);  
  }
  
  // Check data + write angle
  if (readString.length() >0) 
  {
    Serial.println(readString);  //so you can see the captured string 
    int n = readString.toInt();  //convert readString into a number

    // auto select appropriate value, copied from someone elses code.
    if(n > 180 || n < 0)
    {
      Serial.println("Not good value");
    }
    else
    {   
      Serial.print("writing Angle: ");
      Serial.println(n);
      servomotor.write(n);
      servomotor2.write(n);
      servomotor3.write(n);
    }

    readString=""; //empty for next input
  } 
  
}
