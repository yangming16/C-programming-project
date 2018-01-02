//BUG 2.1 G(80,0) define the original orientation of the car to be the x coordinate of the spatial frame
//BUG2.2 G(10,30)
#include <RedBot.h>
RedBotSensor left = RedBotSensor(A3);
RedBotSensor center = RedBotSensor(A6); 
RedBotSensor right = RedBotSensor(A7); 
RedBotAccel accelerometer;
#define LINETHRESHOLD 800
#define SPEED 80



RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A2, 10); 
int buttonPin = 12;
int leftSpeed;   
int rightSpeed; 
float lCount;
float rCount;
float def=0; 
float C=20.428;
float a=0;
float r=16.1;
float x0=0;
float y0=0;
float x1=0;
float y1=0;
float lc1=0;
float rc1=0;
float lc2=0;
float rc2=0;
int x_hit=0;
int y_hit=0;
int x_leave=0;
int y_leave = 0;
int i = 0;
float theta=0;
float x=40;
float y =-40;// coordinate of the goal
float def1=y/x;
float def2=0;// slope of loction of car and goal
int k = 0 ;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
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

    
if (((abs(x-x0)<10)&&(abs(y-y0)<10))==0) // not reach the goal
{
     if((center.read() < LINETHRESHOLD)&&(right.read() < LINETHRESHOLD)&&(left.read() < LINETHRESHOLD))// no obstacle , move toward the goal
  {
   if (def>10)
   {
    leftSpeed = SPEED/2;
    rightSpeed = SPEED ;
    motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);

   }//turn left
   else if (def<-10)
   {
    leftSpeed = -SPEED ;
    rightSpeed = -SPEED/2;
   motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
 
   }//turn right
   else if (abs(def)<10)
   {
    leftSpeed = -SPEED; 
    rightSpeed = SPEED;
    motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
    delay(0);
    }//go straight
} // no obstacle, adjust car's orientation to move toward the goal
else if ((((center.read() < LINETHRESHOLD)&&(right.read() < LINETHRESHOLD)&&(left.read() < LINETHRESHOLD))==0)&&(((center.read() > LINETHRESHOLD)&&(right.read() > LINETHRESHOLD)&&(left.read() > LINETHRESHOLD))==0)) 
     //encounter obstacle,tracing
{
  if ((((abs(def2-def1))<10)&&(abs(tan(theta)-def1)>50))==0)// not p(leave)
  {

  if((center.read() > LINETHRESHOLD)&&(right.read() < LINETHRESHOLD)&&(left.read() < LINETHRESHOLD))
  {
    leftSpeed = -SPEED; 
    rightSpeed = SPEED;
    motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
        delay(0);   
  }
 //go straight
  
  else if((center.read() < LINETHRESHOLD)&&(right.read() > LINETHRESHOLD)&&(left.read() < LINETHRESHOLD))
  {
    leftSpeed = -SPEED;
    rightSpeed = -SPEED/2;
       motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
        delay(0);  
  }
 //go right

  else if((center.read() < LINETHRESHOLD)&&(right.read() < LINETHRESHOLD)&&(left.read() > LINETHRESHOLD))
  {
    leftSpeed = SPEED/2;
    rightSpeed = SPEED;
    motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
        delay(0);  
  }
 //go left
   else if((center.read() > LINETHRESHOLD)&&(right.read() < LINETHRESHOLD)&&(left.read() > LINETHRESHOLD))
  {
    leftSpeed = SPEED/2;
    rightSpeed = SPEED ;
        motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
        delay(0);  
  }
 //go left

  else if((center.read() > LINETHRESHOLD)&&(right.read() > LINETHRESHOLD)&&(left.read() < LINETHRESHOLD))
  {
    leftSpeed = -SPEED;
    rightSpeed = -SPEED/2;
    motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);
        delay(0);  
  }
 //go right
  delay(0);
  }
  else if (((abs(def2-def1))<10)&&(abs(tan(theta)-def1)>50))// time to leave
{
   leftSpeed = -SPEED;
   rightSpeed = -SPEED/2;
   motors.rightMotor(rightSpeed);
   motors.leftMotor(leftSpeed);
   delay(2000);   
  }//depart,move toward the goal,turn right
  
   }
   else if ((center.read() > LINETHRESHOLD)&&(right.read() > LINETHRESHOLD)&&(left.read() > LINETHRESHOLD))
{
if (abs(def)>5)
{
    leftSpeed = -SPEED; 
    rightSpeed = SPEED;
    motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);  
        delay(0);   
  }//go straight
if (abs(def)<5)
{
    leftSpeed = -SPEED;
    rightSpeed = -SPEED/2;
    motors.rightMotor(rightSpeed);
    motors.leftMotor(leftSpeed);  
    delay(1000);
  }// encounter obstacle, turn right
}
}



 // now, we will start to determine the orientation and position of the car real time

    def2 = (y-y0)/(x-x0);
    lCount = encoder.getTicks(LEFT);
    rCount = encoder.getTicks(RIGHT); 
    a=(rCount-lCount)*C/192;
    theta=a/r; // determine the theta of the car
    if (theta>2*PI)
    theta=theta-2*PI;
    if(theta<-2*PI)
    theta=theta+2*PI;
    def = y-x*tan(theta)+x0*tan(theta)-y0; // the function which is determined by the coordinate and orientation of the car
    if (leftSpeed==-rightSpeed)
    {
    lc1 = encoder.getTicks(LEFT);
    x0=x0+(lc1-lc2)*C*cos(theta)/192;
    y0=y0+(lc1-lc2)*C*sin(theta)/192;   
    lc2=lc1;
      }// when car moving forward, add up the displacement
    
      
   Serial.print(x0);
   Serial.print("\t");  
   Serial.print(y0);
   Serial.print("\t");
   Serial.print(theta);
   Serial.print("\t");
   Serial.print(def);
   Serial.print("\t");
   Serial.print(left.read());
  Serial.print("\t");  // tab character
  Serial.print(center.read());
  Serial.print("\t");  // tab character
  Serial.print(right.read());
  Serial.print("\t");  // tab character
   Serial.println();
   // debug

    if ((abs((x0-x)<10))&&(abs((y-y0)<10)))
    motors.brake();//reach the goal,stop
}
}









