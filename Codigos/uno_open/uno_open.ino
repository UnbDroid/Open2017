#include "roscom.h"
#include "MotorsEnc.h"
#include <TaskScheduler.h>

void messageInt32Cb( const arduino_msgs::StampedInt32& r_int32_msg)
{ 
    
}
void messageFloat32Cb( const arduino_msgs::StampedFloat32& r_float32_msg)
{
  
}
//8 sensores US
//9 sensores de toque

int gooooo;
void setup()
{
  SetupInterrupt();
  initializeROS();
  pinMode(LED_BUILTIN, OUTPUT);
  gooooo =0;
  Serial.begin(9600);
  
}

void loop()
{
  /*sendInt32(0,5);
  sendFloat32(1,5.55);
  nh.spinOnce();*/
  if( DegreeToCm(Degree(encCountLeft[0])) <40 && DegreeToCm(Degree(encCountRight[0])) <40 && gooooo== 0){
    Serial.println(DegreeToCm(Degree(encCountRight[0])));
    OnFwd(MOTOR_RIGHT,POT);
    OnFwd(MOTOR_LEFT,POT);
  }else{
    OffMotors();
    gooooo = 1;
  }
}
