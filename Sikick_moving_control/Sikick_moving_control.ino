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

int dir = 8;
int pwm = 11;
int val = 0;

int Steering_Pin = 10;

int angle = 90; // Init angle

void drive_pwm_cb(const std_msgs::UInt16MultiArray& cmd_msg) {
  val = cmd_msg.data[0];  // set drive PWM
  digitalWrite(dir, LOW);
  analogWrite(pwm, val);
  digitalWrite(13, HIGH-digitalRead(13));  // toggle led
}

void steer_angle_cb(const std_msgs::UInt16MultiArray& cmd_msg) {
  angle = cmd_msg.data[1];  // set servo angle, should be from 0-180 
  servo.write(angle);
  digitalWrite(13, HIGH-digitalRead(13));  // toggle led
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub_drive("speed_drive_pwm", drive_pwm_cb);
ros::Subscriber<std_msgs::UInt16MultiArray> sub_steer("steer_angle_pwm", steer_angle_cb);

void setup() {
  pinMode(13, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(pwm, OUTPUT);

  nh.initNode();
  nh.subscribe(sub_drive);
  nh.subscribe(sub_steer);
  servo.attach(Steering_Pin);  // Pin number
}

void loop() {
  nh.spinOnce();
  delay(1);
}
