#include <math.h>

#define POS 1
#define NEG 0
#define CCW 1
#define CW 0
#define RADIANS_PER_DEGREE 3.141592/180.0
#define DEGREES_PER_RADIAN 180.0/3.141592
#define PERIOD 4000
#define TIMEOUT 5000
#define CAM_GARRA 0
#define CAM_MAMADOR 1
#define ENDSTOP_OFFSET 30

byte stepper_cam[] = {2, 3, 4, 5};
byte current_state = 0;
bool endstop = 0;
long int last_step_time;
bool cam_state;
int i;

void initStepper(byte in[4]) {
  pinMode(in[0], OUTPUT);
  pinMode(in[1], OUTPUT);
  pinMode(in[2], OUTPUT);
  pinMode(in[3], OUTPUT);
  digitalWrite(in[0], HIGH);
  digitalWrite(in[1], LOW);
  digitalWrite(in[2], LOW);
  digitalWrite(in[3], HIGH);
  delay(100);
}

void oneStep(byte pin[4], byte* current_state, bool dir) {
  if(dir == POS){  
    switch(*current_state) {
      case 0:
        *current_state = 3;
        break;
      case 1:
        *current_state = 0;
        break;
      case 2:
        *current_state = 1;
        break;
      case 3:
        *current_state = 2;
        break;
    }
  } else if(dir == NEG) {
    switch(*current_state) {
      case 0:
        *current_state = 1;
        break;
      case 1:
        *current_state = 2;
        break;
      case 2:
        *current_state = 3;
        break;
      case 3:
        *current_state = 0;
        break;
    }
  }
  switch(*current_state) {
    case 0:
      digitalWrite(pin[0], HIGH);
      digitalWrite(pin[1], LOW);
      digitalWrite(pin[2], LOW);
      digitalWrite(pin[3], HIGH);
      break;
    case 1:
      digitalWrite(pin[0], HIGH);
      digitalWrite(pin[1], HIGH);
      digitalWrite(pin[2], LOW);
      digitalWrite(pin[3], LOW);
      break;
    case 2:
      digitalWrite(pin[0], LOW);
      digitalWrite(pin[1], HIGH);
      digitalWrite(pin[2], HIGH);
      digitalWrite(pin[3], LOW);
      break;
    case 3:
      digitalWrite(pin[0], LOW);
      digitalWrite(pin[1], LOW);
      digitalWrite(pin[2], HIGH);
      digitalWrite(pin[3], HIGH);
      break;
  }
}

void InitCamera()
{
	initStepper(stepper_cam);
	while(endstop == 0)
	{
		oneStep(stepper_cam, &current_state, POS);
		last_step_time = micros();
		while(micros() - last_step_time < PERIOD){
			//espaço pra fazer coisas durante o espaço entre os passos
		}
	}
	for(i = 0; i < ENDSTOP_OFFSET; ++i)
	{
		oneStep(stepper_cam, &current_state, NEG);
		last_step_time = micros();
		while(micros() - last_step_time < PERIOD){
			//espaço pra fazer coisas durante o espaço entre os passos
		}
	}
	cam_state = CAM_MAMADOR;
}

void ChangeCamState()
{
	bool dir;
	if(cam_state == CAM_MAMADOR)
	{
		dir = NEG;
		cam_state = CAM_GARRA;
	} else {
		dir = POS;
		cam_state = CAM_MAMADOR;
	}
	for(i = 0; i < 1024; ++i)
	{
		oneStep(stepper_cam, &current_state, dir);
		last_step_time = micros();
		while(micros() - last_step_time < PERIOD){
			//espaço pra fazer coisas durante o espaço entre os passos
		}
	}
}
