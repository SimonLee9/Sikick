#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h> // Servo motor
#include <ros.h> // for cmd_vel control
#include <std_msgs/UInt16MultiArray.h> // for cmd_vel control

ros::NodeHandle  nh;


////// Pin Map /////
int Lidar_ServoPin = 13; // Lidar servo
int Steering_dir = 8; // check
int Steering_pwm = 11;
int Steering_Pin = 9;

////// Setup Parameter Name /////
Servo Lidar_servo;  //Lidar servo
Servo Steering_servo;

////// Parameter Value  /////
int Lidar_angle = 50; // servo position in degrees 
int Lidar_min_angle = 40;
int Lidar_max_angle = 73;

int Steering_val = 0;
int Steering_angle = 90; // Init angle

int meter5= 60; 
//25 = 1.57m
//30 = 1.7m
//40 = 2m
//50 = 2.75m
//60 = 4.6m

////// Time  /////
int Lidar_scanning_delay_time=500;



void Steering_servo_cb( const std_msgs::UInt16MultiArray& cmd_msg){
  Steering_val = cmd_msg.data[0]; //set servo angle, should be from 0-180 
  Steering_angle = cmd_msg.data[1];
  Steering_servo.write(Steering_angle);
  digitalWrite(13, HIGH-digitalRead(13));//toggle led
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("control", Steering_servo_cb);

void setup() 
{ 
    Lidar_servo.attach(Lidar_ServoPin);

    Steering_servo.attach(Steering_Pin); // Pin number

    pinMode(13, OUTPUT);
    pinMode(Steering_dir,OUTPUT);
    pinMode(Steering_pwm,OUTPUT);

    nh.initNode();
    nh.subscribe(sub);
    

} 

void scanning()
{

  // scan from 0 to 180 degrees
  for(Lidar_angle = 65; Lidar_angle < 73; Lidar_angle++) 
  //for(min_angle; min_angle < max_angle; min_angle++)
  { 
    Lidar_servo.write(Lidar_angle);
    //servo2.write(angle); 
    delay(Lidar_scanning_delay_time); //servo motor ¼Óµµ Á¦¾î
  } 
  // now scan back from 180 to 0 degrees
  for(Lidar_angle = 73; Lidar_angle >65; Lidar_angle--)
  //for(max_angle; max_angle >min_angle; max_angle--) 
  { 
    Lidar_servo.write(Lidar_angle);
    //servo2.write(angle);
    delay(Lidar_scanning_delay_time); //servo motor ¼Óµµ Á¦¾î
  }
}

void Xmeter()
{
  Lidar_servo.write(meter5);
}

void loop() 
{ 
   //scanning();
   //Xmeter();

    digitalWrite(Steering_dir,LOW);
    analogWrite(Steering_pwm,Steering_val); 
    nh.spinOnce();
    delay(1);
}
