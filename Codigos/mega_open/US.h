#ifndef ULTRASSOM_H
#define ULTRASSOM_H

#include <NewPing.h>


#define FRENTE_R 0
#define FRENTE_L 0
#define TRAS_R 0
#define TRAS_L 0
#define LADO_R 0
#define LADO_L 0
#define CIMA_R 0
#define CIMA_L 0


#define NUM_IDEN_US 100

#define TRIGGER_1 26 
#define ECHO_1 28 
#define TRIGGER_2 24
#define ECHO_2 22
#define TRIGGER_3 25
#define ECHO_3 23
#define TRIGGER_4 29
#define ECHO_4 27
#define TRIGGER_5 33
#define ECHO_5 31
#define TRIGGER_6 37
#define ECHO_6 35
#define TRIGGER_7 39
#define ECHO_7 41
#define TRIGGER_8 43 
#define ECHO_8 45

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define SONAR_NUM  8 // Number of sensors.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(TRIGGER_1, ECHO_1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIGGER_2, ECHO_2, MAX_DISTANCE),
  NewPing(TRIGGER_3, ECHO_3, MAX_DISTANCE),
  NewPing(TRIGGER_4, ECHO_4, MAX_DISTANCE),
  NewPing(TRIGGER_5, ECHO_5, MAX_DISTANCE),
  NewPing(TRIGGER_6, ECHO_6, MAX_DISTANCE), 
  NewPing(TRIGGER_7, ECHO_7, MAX_DISTANCE),
  NewPing(TRIGGER_8, ECHO_8, MAX_DISTANCE)
};
int USReadings[SONAR_NUM];

void readUS(int us_num){
  USReadings[us_num] = sonar[us_num].ping_cm();
}

#endif
