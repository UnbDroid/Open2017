#ifndef ROSCOM_H
#define ROSCOM_H
#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>

#include <arduino_msgs/StampedInt32.h>
#include <arduino_msgs/StampedFloat32.h>


ros::NodeHandle nh;

arduino_msgs::StampedInt32 int32_msg;
arduino_msgs::StampedFloat32 float32_msg;

ros::Publisher int32_p("arduinoN_int32",&int32_msg);
ros::Publisher float32_p("arduinoN_float32",&float32_msg);

void messageInt32Cb( const arduino_msgs::StampedInt32& r_int32_msg);
void messageFloat32Cb( const arduino_msgs::StampedFloat32& r_float32_msg);
double t0 =0 , t1=0;

ros::Subscriber<arduino_msgs::StampedInt32> subInt32("raspberryN_int32", &messageInt32Cb );
ros::Subscriber<arduino_msgs::StampedFloat32> subFloat32("raspberryN_float32", &messageFloat32Cb );

void initializeROS()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(int32_p);
  nh.advertise(float32_p);
  
  nh.subscribe(subInt32);
  nh.subscribe(subFloat32);
}

void sendInt32(long int id, long int data)
{
   int32_msg.id = id;
   int32_msg.data = data;
   int32_p.publish(&int32_msg);  
}

void sendFloat32(int id, float data)
{
   float32_msg.id = id;
   float32_msg.data = data;
   float32_p.publish(&float32_msg);
}

#endif
