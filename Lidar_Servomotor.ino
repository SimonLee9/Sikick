#include <Servo.h> 
 
int servoPin1 = 10;
//int servoPin2 = 13;

Servo servo1; 
//Servo servo2;

int angle = 50; // servo position in degrees 
int min_angle = 40;
int max_angle = 73;

int scanning_delay_time=100;

int meter5= 65; 

int init_angle = 90;

//25 = 1.57m
//30 = 1.7m
//40 = 2m
//50 = 2.75m
//60 = 4.6m
void setup() 
{ 
    servo1.attach(servoPin1);
    //servo2.attach(servoPin2);
} 

void scanning()
{

  // scan from 0 to 180 degrees
  for(angle = 75; angle < 125; angle++) 
  //for(min_angle; min_angle < max_angle; min_angle++)
  { 
    servo1.write(angle);
    //servo2.write(angle); 
    delay(scanning_delay_time); //servo motor ¼Óµµ Á¦¾î
  } 
  // now scan back from 180 to 0 degrees
  for(angle = 125; angle >75; angle--)
  //for(max_angle; max_angle >min_angle; max_angle--) 
  { 
    servo1.write(angle);
    //servo2.write(angle);
    delay(scanning_delay_time); //servo motor ¼Óµµ Á¦¾î
  }
}

void Xmeter()
{
  servo1.write(meter5);
}

void Center()
{
  servo1.write(init_angle);
  delay(scanning_delay_time);
}

void loop() 
{ 
   //scanning();
   //Xmeter();
   Center();
}
