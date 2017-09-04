#include "ros/ros.h"
#include "arduino_msgs/StampedUint8.h"
#include "arduino_msgs/StampedUint32.h"
#include "arduino_msgs/StampedUint64.h"
#include "arduino_msgs/StampedInt8.h"
#include "arduino_msgs/StampedInt32.h"
#include "arduino_msgs/StampedInt64.h"
#include "arduino_msgs/StampedBool.h"
#include "arduino_msgs/StampedChar.h"
#include "arduino_msgs/StampedFloat32.h"
#include "arduino_msgs/StampedFloat64.h"
#include "arduino_msgs/StampedString.h"
#include <sstream>
#include <math.h>

void messageMInt64Cb( const arduino_msgs::StampedInt64& aM_int64_msg)
{
}
void messageMFloat64Cb( const arduino_msgs::StampedFloat64& aM_float64_msg)
{
}
void messageMFloat32Cb( const arduino_msgs::StampedFloat32& aM_float32_msg)
{
}
void messageNInt32Cb( const arduino_msgs::StampedInt32& aN_int32_msg)
{
}
void messageNFloat32Cb( const arduino_msgs::StampedFloat32& aN_float32_msg)
{
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "first_test");
	ros::NodeHandle nh;
	/*-------------------------------------------settar as paradas do arduino mega----------------------------------------*/
	ros::Publisher pubM_int64 = nh.advertise<arduino_msgs::StampedInt64>("raspberryM_int64", 1000);
	ros::Publisher pubM_float64 = nh.advertise<arduino_msgs::StampedFloat64>("raspberryM_float64", 1000);
	ros::Publisher pubM_float32 = nh.advertise<arduino_msgs::StampedFloat32>("raspberryM_float32", 1000);
	ros::Subscriber subM_int64 = nh.subscribe("arduinoM_int64", 1000, messageMInt64Cb);
	ros::Subscriber subM_float64 = nh.subscribe("arduinoM_float32", 1000, messageMFloat64Cb);
	ros::Subscriber subM_float32 = nh.subscribe("arduinoM_float64", 1000, messageMFloat32Cb);
	arduino_msgs::StampedInt64 int64M_msg;	
	arduino_msgs::StampedFloat64 float64M_msg;
	arduino_msgs::StampedFloat32 float32M_msg;
	/*-------------------------------------------settar as paradas do arduino nano---------------------------------------*/
	ros::Publisher pubN_int32 = nh.advertise<arduino_msgs::StampedInt32>("raspberryN_int32", 1000);
	ros::Publisher pubN_float32 = nh.advertise<arduino_msgs::StampedFloat32>("raspberryN_float32", 1000);
	ros::Subscriber subN_int32 = nh.subscribe("arduinoN_int32", 1000, messageNInt32Cb);
	ros::Subscriber subN_float32 = nh.subscribe("arduinoN_float32", 1000, messageNFloat32Cb);
	arduino_msgs::StampedInt32 int32N_msg;	
	arduino_msgs::StampedFloat32 float32N_msg;
	
	ros::Rate loop_rate(10);	
	int count = 0;
	while (ros::ok())
	{
		int64M_msg.data = count;
		float32M_msg.data = count;
		float64M_msg.data = count;
		int32N_msg.data = count;
		float32N_msg.data = count;
		pubM_int64.publish(int64M_msg);
		pubM_float64.publish(float64M_msg);
		pubM_float32.publish(float32M_msg);
		pubN_int32.publish(int32N_msg);
		pubN_float32.publish(float32N_msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
