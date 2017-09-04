#include "roscom.h"

void messageInt32Cb( const arduino_msgs::StampedInt32& r_int32_msg)
{   
}
void messageFloat32Cb( const arduino_msgs::StampedFloat32& r_float32_msg)
{
}


void setup()
{
  initializeROS();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  sendInt32(0,5);
  sendFloat32(1,5.55);
  nh.spinOnce();                       
}
