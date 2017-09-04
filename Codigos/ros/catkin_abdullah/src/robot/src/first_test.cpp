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

void messageInt64Cb( const arduino_msgs::StampedInt64& a_int64_msg)
{
}

void messageFloat64Cb( const arduino_msgs::StampedFloat64& a_float64_msg)
{
}

void messageFloat32Cb( const arduino_msgs::StampedFloat32& a_float32_msg)
{
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "first_test");
	ros::NodeHandle nh;
	ros::Publisher pub_int64 = nh.advertise<arduino_msgs::StampedInt64>("raspberry_int64", 1000);
	ros::Publisher pub_float64 = nh.advertise<arduino_msgs::StampedFloat64>("raspberry_float64", 1000);
	ros::Publisher pub_float32 = nh.advertise<arduino_msgs::StampedFloat32>("raspberry_float32", 1000);
	ros::Subscriber sub_int64 = nh.subscribe("arduino_int64", 1000, messageInt64Cb);
	ros::Subscriber sub_float64 = nh.subscribe("arduino_float32", 1000, messageFloat64Cb);
	ros::Subscriber sub_float32 = nh.subscribe("arduino_float64", 1000, messageFloat32Cb);
	ros::Rate loop_rate(10);
	arduino_msgs::StampedInt64 int64_msg;	
	arduino_msgs::StampedFloat64 float64_msg;
	arduino_msgs::StampedFloat32 float32_msg;
	int count = 0;
	while (ros::ok())
	{
		int64_msg.data = count;
		float32_msg.data = count + count/(pow(10,int(count/10)+1));
		float64_msg.data = count + 1 + count/(pow(10,int(count/10)+1));
		pub_int64.publish(int64_msg);
		pub_float64.publish(float64_msg);
		pub_float32.publish(float32_msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
