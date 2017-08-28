#include "roscom.h"

void setup()
{
  initializeROS();
}

void loop()
{
  sendInt64(ReadUS(),1);
  //delay(50);
}
