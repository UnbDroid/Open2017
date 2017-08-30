#ifndef ROSCOM_H
#define ROSCOM_H
#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>

#include <arduino_msgs/StampedUint8.h>
#include <arduino_msgs/StampedUint32.h>
#include <arduino_msgs/StampedUint64.h>
#include <arduino_msgs/StampedInt8.h>
#include <arduino_msgs/StampedInt32.h>
#include <arduino_msgs/StampedInt64.h>
#include <arduino_msgs/StampedBool.h>
#include <arduino_msgs/StampedChar.h>
#include <arduino_msgs/StampedFloat32.h>
#include <arduino_msgs/StampedFloat64.h>
#include <arduino_msgs/StampedString.h>
int testcom;
ros::NodeHandle  nh;
arduino_msgs::StampedInt64 int64_msg;
arduino_msgs::StampedFloat32 float32_msg;
arduino_msgs::StampedFloat64 float64_msg;

ros::Publisher int64_p("arduino_int64",&int64_msg);
ros::Publisher float32_p("arduino_float32",&float32_msg);
ros::Publisher float64_p("arduino_float64",&float64_msg);

void messageInt64Cb( const arduino_msgs::StampedInt64& r_int64_msg);
void messageFloat32Cb( const arduino_msgs::StampedFloat32& r_float32_msg);
void messageFloat64Cb( const arduino_msgs::StampedFloat64& r_float64_msg);

void messageInt64Cb( const arduino_msgs::StampedInt64& r_int64_msg){
 testcom++;
}

void messageFloat32Cb( const arduino_msgs::StampedFloat32& r_float32_msg){
}

void messageFloat64Cb( const arduino_msgs::StampedFloat64& r_float64_msg){
}
ros::Subscriber<arduino_msgs::StampedInt64> subInt64("raspberry_int64", &messageInt64Cb );
ros::Subscriber<arduino_msgs::StampedFloat32> subFloat32("raspberry_float32", &messageFloat32Cb );
ros::Subscriber<arduino_msgs::StampedFloat64> subFloat64("raspberry_float64", &messageFloat64Cb );
//*/



void initializeROS()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(int64_p);
  nh.advertise(float32_p);
  nh.advertise(float64_p);
  
  nh.subscribe(subInt64);
  nh.subscribe(subFloat32);
  nh.subscribe(subFloat64);
}
void sendInt64(int id, int data){
   int64_msg.id = id;
   int64_msg.data = data;
   int64_p.publish(&int64_msg);
}

void sendFloat32(int id, float data){
   float32_msg.id = id;
   float32_msg.data = data;
   float32_p.publish(&float32_msg);
}

void sendFloat64(int id, double data){
   float64_msg.id = id;
   float64_msg.data = data;
   float64_p.publish(&float64_msg);
}
#endif
