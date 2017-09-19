#include "roscom.h"
#include "US.h"
void messageInt64Cb( const arduino_msgs::StampedInt64& r_int64_msg){     
}

void messageFloat32Cb( const arduino_msgs::StampedFloat32& r_float32_msg){
}

void messageFloat64Cb( const arduino_msgs::StampedFloat64& r_float64_msg){
}
//8 sensores US
//9 sensores de toque

void setup()
{
  initializeROS();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  readUS(SONAR_NUM);
  sendInt64(0,USReadings[SONAR_NUM]);
  nh.spinOnce();
}
