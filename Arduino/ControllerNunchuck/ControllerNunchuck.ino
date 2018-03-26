#include <Wire.h>
#include <Servo.h>

/* Constants */
const int vccPin = A3;
const int gndPin = A2;
const int dataLength = 6;
static byte rawData[dataLength];
enum nunchuckItems { joyX, joyY, accelX, accelY, accelZ, btnZ, btnC };
Servo servomotors[3];


/* Setup */
void setup() 
{
  pinMode(gndPin, OUTPUT);
  pinMode(vccPin, OUTPUT);
  digitalWrite(gndPin, LOW);
  digitalWrite(vccPin, HIGH);
  delay(100);
  Serial.begin(9600);
  servomotors[0].attach(3, 1000, 2000);
  servomotors[1].attach(5, 1000, 2000);
  servomotors[2].attach(8);
  nunchuckInit();
  
}

/* Loop */
void loop() 
{
  nunchuckRead();
  /*int acceleration = getValue(accelX);
  if((acceleration >= 75) && (acceleration <= 185))
  {
    byte x = map(acceleration, 75, 185, 0, 63);
    Serial.write(x);
    delay(20);
  }*/
  int joystickX = getValue(joyX);
  int joystickY = getValue(joyY);
  int accelerationX = getValue(accelX);
  int accelerationY = getValue(accelY);
  int accelerationZ = getValue(accelZ);
  int buttonC = getValue(btnC);
  int buttonZ = getValue(btnZ);

  if (buttonC == 1) servomotors[2].write(180);
  else servomotors[2].write(0);

  if (joystickX >= 0 & joystickX <= 180)
  {
    servomotors[1].write(joystickX);
  }
  if (joystickY >= 0 & joystickY <= 180)
  {
    servomotors[0].write(joystickY);
  }
  
  
  Serial.print("joyX: "); Serial.print(joystickX);
  Serial.print("\tjoyY: "); Serial.print(joystickY);
  Serial.print("\taccelX: "); Serial.print(accelerationX);
  Serial.print("\taccelY: "); Serial.print(accelerationY);
  Serial.print("\taccelZ: "); Serial.print(accelerationZ);
  Serial.print("\tbtnC: "); Serial.print(buttonC);
  Serial.print("\tbtnZ: "); Serial.println(buttonZ);
  delay(20);
}

void nunchuckInit()
{
  Wire.begin();
  Wire.beginTransmission(0x52);
  Wire.write((byte) 0x40);
  Wire.write((byte) 0x00);
  Wire.endTransmission();
}

static void nunchuckRequest()
{
  Wire.beginTransmission(0x52);
  Wire.write((byte) 0x00);
  Wire.endTransmission();
}

boolean nunchuckRead()
{
  int cnt = 0;
  Wire.requestFrom(0x52, dataLength);
  while(Wire.available())
  {
    rawData[cnt] = nunchuckDecode(Wire.read());
    cnt++;
  }
  nunchuckRequest();
  if(cnt >= dataLength)
    return true;
  else
    return false;
}

static char nunchuckDecode(byte x)
{
  return (x ^ 0x17) + 0x17;
}

int getValue(int item)
{
  if(item <= accelZ)
    return (int) rawData[item];
  else if(item == btnZ)
    return bitRead(rawData[5], 0) ? 0 : 1;
  else if(item == btnC)
    return bitRead(rawData[5], 1) ? 0 : 1;
}
