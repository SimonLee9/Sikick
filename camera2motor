
#include <Arduino.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <ros/time.h>

#include <std_msgs/Header.h>

//buzzer
int speakerPin =8;
int numTones =8;

int alarmOn=392; //int tones[]={261,277,294,311,330,349,370,392};
//int alarmOff=0;

ros::NodeHandle n;

/*
void ZED_msgCallback( const std_msgs::UInt16& cmd_msg){
  digitalWrite(7, HIGH);  //
  digitalWrite(8, LOW);  //
  analogWrite(9, cmd_msg.data); //set motor speed, should be from 0-255    
  //Serial.println(cmd_msg.data);
}
ros::Subscriber<std_msgs::UInt16> sub("/zed2/zed_node/obj_det/objects", ZED_msgCallback);
// "/zed2/zed_node/obj_det/objects": topic name
// ZED_msgCallback : callback function
*/
void ChekcingConnection(){
  n.loginfo("Connected");
  delay(1000);
}

void action_Motor( const std_msgs::UInt16& msg){
  //n.loginfo(msg.data);

  if(msg.data > 99.0)
    {
      digitalWrite(13,HIGH);
      n.loginfo("Keep Going");

      //Buzzer
      noTone(speakerPin);
      delay(10);
    }
  else if(msg.data < 51.0)
    {
      digitalWrite(13,LOW);
      n.loginfo("Slow");
      
      //Buzzer
      tone(speakerPin,alarmOn);
      delay(10);
    }
  analogWrite(9, msg.data); //set motor speed, should be from 0-255
}



ros::Subscriber<std_msgs::UInt16> sub("/ZED_detection", &action_Motor);

void setup(){
  pinMode(13, OUTPUT);
  //pinMode(7, OUTPUT); // direction 1
  //pinMode(8, OUTPUT); // direction 2
  pinMode(9, OUTPUT); // speed

  n.initNode();
  n.subscribe(sub);
}


void loop(){
  n.spinOnce();
 // n.spin();
  
  //ChekcingConnection();
  //action_Motor;
  
  //n.loginfo(msg.data);
  
  delay(100);
}
