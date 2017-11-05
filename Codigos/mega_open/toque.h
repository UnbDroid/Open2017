#ifndef SENSORDETOQUE_H
#define SENSORDETOQUE_H


#define QUANTIDADE_SENSOR_TOQUE 5 
#define NUM_IDEN_TOQUE 200
#define pin1 1
#define pin2 2
#define pin3 3
#define pin4 4
#define pin5 5
#define pin6 6
#define pin7 7
#define pin8 8
#define pin9 9

bool toque[QUANTIDADE_SENSOR_TOQUE];
bool oldToque[QUANTIDADE_SENSOR_TOQUE];

void startSENSORTOQUE(){
	pinMode(pin1,INPUT);
	pinMode(pin2,INPUT);
	pinMode(pin3,INPUT);
	pinMode(pin4,INPUT);
	pinMode(pin5,INPUT);
	pinMode(pin6,INPUT);
	pinMode(pin7,INPUT);
	pinMode(pin8,INPUT);
	pinMode(pin9,INPUT);
	int i = 0 ;
	while(i<QUANTIDADE_SENSOR_TOQUE){
		toque[i] = false;    oldToque[i] = false;     i++;
	} 

}

void lerSensoresToque(){
	int i = 0;
	while(i<QUANTIDADE_SENSOR_TOQUE)
	{
		toque[i] = digitalRead(i+1);	i++;
	} 	
}

#endif


