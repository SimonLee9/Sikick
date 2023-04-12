#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
//#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo;

int dir = 8;
int pwm =11;
int vel = 0;

int Steering_Pin = 10;

int angle = 90; // Init angle

int speakerPin = 12; //Buzzer

int alaramOff=0;
int alaramOn = 400;

int alaramCam;

void alarm_cb(const std_msgs::UInt16& buz_msg)
{
  /*
  //operation 
  alaramOff =buz_msg.data; 
  tone(speakerPin,alaramOn);
  */
  alaramCam =buz_msg.data; 
  if (alaramCam==392)
  {
    tone(speakerPin,alaramOn);
  }
  else
  {
    tone(speakerPin,alaramOn);
  }
  
  //tone(speakerPin,buz_msg.data);
  //delay(500);
  
}

void servo_cb( const std_msgs::UInt16& cmd_msg){
  //vel = cmd_msg.data[0]; //set servo angle, should be from 0-180 
  //angle = cmd_msg.data[1];

  vel = cmd_msg.data;
  servo.write(angle);
  digitalWrite(13, HIGH-digitalRead(13));//toggle led
}

ros::Subscriber<std_msgs::UInt16> sub("control", servo_cb);
ros::Subscriber<std_msgs::UInt16> sub2("alarm", alarm_cb);

void setup(){
  pinMode(13, OUTPUT);
  pinMode(dir,OUTPUT);
  pinMode(pwm,OUTPUT);
  pinMode(speakerPin,OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  servo.attach(Steering_Pin); // Pin number
}


void loop(){
  digitalWrite(dir,LOW);
  analogWrite(pwm,vel); 

  nh.spinOnce();
  delay(1);
}
