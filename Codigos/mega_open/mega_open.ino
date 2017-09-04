#include "roscom.h"

void setup()
{
  initializeROS();
  pinMode(LED_BUILTIN, OUTPUT);
  cont_test_com = 0;
}

void loop()
{
  sendInt64(0,ReadUS());
  if(cont_test_com>1000){
     digitalWrite(LED_BUILTIN, HIGH);
     delay(500);
     cont_test_com = 0;
  }else{
     digitalWrite(LED_BUILTIN, LOW);  
  }
  nh.spinOnce();
  //delay(50);
}
