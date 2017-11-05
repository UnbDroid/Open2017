#ifndef ULTRASSOM_H
#define ULTRASSOM_H

#include <NewPing.h>


#define FRENTE_R 0
#define FRENTE_L 1
#define TRAS_R 2
#define TRAS_L 3
#define LADO_R 4
#define LADO_L 5
#define CIMA_R 6
#define CIMA_L 7


#define NUM_IDEN_US 100

#define TRIGGER_1 51 
#define ECHO_1 53 

#define TRIGGER_2 47
#define ECHO_2 49

#define TRIGGER_3 43
#define ECHO_3 45

#define TRIGGER_4 39
#define ECHO_4 41

#define TRIGGER_5 35
#define ECHO_5 37

#define TRIGGER_6 31
#define ECHO_6 33

#define TRIGGER_7 29
#define ECHO_7 29

#define TRIGGER_8 23 
#define ECHO_8 25

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
