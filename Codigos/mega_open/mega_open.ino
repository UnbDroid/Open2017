#include "roscom.h"

void messageInt64Cb( const arduino_msgs::StampedInt64& r_int64_msg){     
}

void messageFloat32Cb( const arduino_msgs::StampedFloat32& r_float32_msg){
}

void messageFloat64Cb( const arduino_msgs::StampedFloat64& r_float64_msg){
}

void setup()
{
  initializeROS();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  sendInt64(0,ReadUS());
  nh.spinOnce();
  delay(50);
}
