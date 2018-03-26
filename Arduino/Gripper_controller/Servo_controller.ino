#include <Servo.h>

// Stream buffer
String readString;

// Gripper
Servo gripper;

// Angle variable
unsigned char angle = 0;


void setup() 
{
  // Initialize the serial link
  Serial.begin(9600);

  // Attahc the gripper
  gripper.attach(2, 1000, 2000);
}

void loop() 
{
  
  while (Serial.available()) 
  {
    // read the incoming byte:
    char c = Serial.read();
    readString += c;
    delay(2);  
  }

  
  if (readString.length() >0) 
  {
    Serial.println(readString);  //so you can see the captured string 
    int n = readString.toInt();  //convert readString into a number

    // auto select appropriate value, copied from someone elses code.
    if(n > 180 || n < 0)
    {
      Serial.println("Too low value");
    }
    else
    {   
      Serial.print("writing Angle: ");
      Serial.println(n);
      gripper.write(n);
    }

    readString=""; //empty for next input
  } 
  


}
