#include <Wire.h>
#include <MPU6050.h>

#define GIRA 300

#define VEL_GIRO_DIR 172.5f
#define VEL_GIRO_ESQ 172.5f
#define DEBUG_INICIO_GIRO 9999
#define ACABOU_GIRO 1
#define ANGULO_ATUAL 2 

bool gDir;
bool InicioDoGiro;
bool STATE = true;
double graus;

MPU6050 mpu;

unsigned long timer = 0;
float timeStep = 0.01;

float pitch = 0;
float roll = 0;
float yaw = 0;

 void inicializaGiro() 
{
    long int t1,t0 = millis();
    while((!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)))// && (t1 - t0) < 3000 )
    {
     // t1 = millis();
      
      delay(500);
    }
    //if(t1 - t0 < 4300){
      mpu.calibrateGyro();
      mpu.setThreshold(3);
      InicioDoGiro = 1;
     // return 0;
    //}else
     // return 1;
}

void atualizaGiro()
{
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;
  delay((timeStep*1000) - (millis() - timer));
}

bool casoDoGiro()
{
   //testar para valores e direcoes diferentes
  if((graus<0 && gDir == 1) || (graus>0 && gDir == 0) )
    return 1;
  else if((graus<0 && gDir == 0) || (graus>0 && gDir == 1) )
    return 0;
}

void Turn(){
  if(InicioDoGiro)
  {
    //noInterrupts();
    settavaloresIniciaisParametros();
    detachInterrupt(digitalPinToInterrupt(encoder0PinA));
    detachInterrupt(digitalPinToInterrupt(encoder1PinA));
    InicioDoGiro=0; 
    pitch = 0;
    roll = 0;
	  yaw = 0;
  }
  atualizaGiro();
  if(abs(roll)<abs(graus))
  {
    atualizaGiro();
    sendInt32(ANGULO_ATUAL, roll);
    if(casoDoGiro()){
      esquerdaEixo(VEL_GIRO_ESQ, VEL_GIRO_DIR);
    }else{
      direitaEixo(VEL_GIRO_ESQ, VEL_GIRO_DIR);
    }
  }else
  {
    //interrupts(); 
    travar();
    delay(100);
    settavaloresIniciaisParametros();
    attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
    STATE = 1;
    InicioDoGiro = 1;
    sendInt32(ACABOU_GIRO,0);
  }
}
