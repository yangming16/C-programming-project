#include <RedBot.h>
RedBotSensor left = RedBotSensor(A3);   
RedBotSensor center = RedBotSensor(A6); 
RedBotSensor right = RedBotSensor(A7); 
RedBotAccel accelerometer;
#define LINETHRESHOLD 800
#define SPEED 50 

RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A2, 10); 
int buttonPin = 12;
int countsPerRev = 192;  
int leftSpeed;   
int rightSpeed; 
float lCount;
float rCount;
int i=0;
int k=0;



void setup()
{ 
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.println("Welcome to group 9 project!");
  Serial.println("------------------------------------------");
  delay(20);
  Serial.println("IR Sensor Readings: ");
  delay(10);
}

void loop()
{
  accelerometer.read();
  if(abs( accelerometer.x)>1000)
  k++;
  if(k>0)
  {
  if(i==0)
  {
    encoder.clearEnc(BOTH); 
  i++;
    }

  Serial.print(left.read());
  Serial.print("\t");  // tab character
  Serial.print(center.read());
  Serial.print("\t");  // tab character
  Serial.print(right.read());
  Serial.print("\t");  // tab character
  
 
  if((center.read() > LINETHRESHOLD)&&(right.read() < LINETHRESHOLD)&&(left.read() < LINETHRESHOLD))
  {
    leftSpeed = -SPEED; 
    rightSpeed = SPEED;
    motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
  }
 //go straight
  
  else if((center.read() < LINETHRESHOLD)&&(right.read() > LINETHRESHOLD)&&(left.read() < LINETHRESHOLD))
  {
    leftSpeed = -(SPEED + 30);
    rightSpeed = 0;
       motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
  }
 //go right

  else if((center.read() < LINETHRESHOLD)&&(right.read() < LINETHRESHOLD)&&(left.read() > LINETHRESHOLD))
  {
    leftSpeed = 0;
    rightSpeed = SPEED + 30;
        motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
  }
 //go left
  
  if((center.read() > LINETHRESHOLD)&&(right.read() > LINETHRESHOLD)&&(left.read() > LINETHRESHOLD))
  {
    leftSpeed = -SPEED; 
    rightSpeed = SPEED;
        motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
  }
 //go straight
    if((center.read() < LINETHRESHOLD)&&(right.read() < LINETHRESHOLD)&&(left.read() < LINETHRESHOLD))
  {
    leftSpeed = 0; 
    rightSpeed = 0;
  }
 //STOP
  else if((center.read() > LINETHRESHOLD)&&(right.read() < LINETHRESHOLD)&&(left.read() > LINETHRESHOLD))
  {
    leftSpeed = 0;
    rightSpeed = SPEED + 30;
     motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
  }
 //go left
  else if((center.read() > LINETHRESHOLD)&&(right.read() > LINETHRESHOLD)&&(left.read() < LINETHRESHOLD))
  {
    leftSpeed = -(SPEED +30);
    rightSpeed = 0;
   motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
  }
 //go right
  delay(0);
 
    lCount = encoder.getTicks(LEFT);
    rCount = encoder.getTicks(RIGHT);
    float j=(lCount+rCount)/(2*192)*(2.56*PI);
  if ((lCount+rCount)/(2*192)*(2.56*PI)>=50)
  motors.brake();
  
int l=j;

  Serial.print(l);
  Serial.println();
  }
}

