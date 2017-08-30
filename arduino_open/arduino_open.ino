#include "roscom.h"

void setup()
{
  initializeROS();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  sendInt64(0,ReadUS());
  if(testcom>850)
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1500);
    testcom = 0;      
    digitalWrite(LED_BUILTIN, LOW); 
  }
  nh.spinOnce();
}
