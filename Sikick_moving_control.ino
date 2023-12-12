//Arduino Nano
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

ros::NodeHandle nh;

Servo servo;

// Arduino Pin map
int buzzer = 12;
int drive_pwm = 11;
int Steering_Pin = 9;

// Init
int val = 0;
int angle = 90; // Init angle

void drive_pwm_cb(const std_msgs::UInt16MultiArray& cmd_msg) {
  val = cmd_msg.data[0];  // set drive PWM
  //digitalWrite(Steering_Pin, LOW);
  analogWrite(drive_pwm, val);

  /*Debug*/
  //digitalWrite(13, HIGH-digitalRead(13));  // toggle led
  //Serial.println("TEst");
  //Serial.println(val);
}

void steer_angle_cb(const std_msgs::UInt16MultiArray& cmd_msg) {
  angle = cmd_msg.data[1];  // set servo angle, should be from 0-180 
  servo.write(angle);

  /*Debug*/
  //digitalWrite(13, HIGH-digitalRead(13));  // toggle led
  //Serial.println(angle);
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub_drive("speed_drive_pwm", drive_pwm_cb);
ros::Subscriber<std_msgs::UInt16MultiArray> sub_steer("steer_angle_pwm", steer_angle_cb);

void beep(){
  digitalWrite(buzzer, HIGH);
  delay(250);
  digitalWrite(buzzer, LOW);
}

void setup() {
  pinMode(13, OUTPUT); // LED
  pinMode(Steering_Pin, OUTPUT);
  pinMode(drive_pwm, OUTPUT);

  nh.initNode();
  nh.subscribe(sub_drive);
  nh.subscribe(sub_steer);
  servo.attach(Steering_Pin);  // Pin number

  //Serial.begin(9600);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
