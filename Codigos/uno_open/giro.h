
#include <Wire.h>
#include <MPU6050.h>

#define GIRA 300

#define VEL_GIRO_DIR 150.0f
#define VEL_GIRO_ESQ 172.5f

#define ACABOU_GIRO 1
#define ANGULO_ATUAL 2 

bool InicioDoGiro;
bool STATE;
int graus;

MPU6050 mpu;


// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

void inicializaGiro() 
{
  Serial.begin(115200);

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  InicioDoGiro = 1;
}

void atualizaGiro()
{
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;
  Serial.print(" Roll = ");
  Serial.print(roll);
  delay((timeStep*1000) - (millis() - timer));
}

void Turn(){
  if(InicioDoGiro){
    travar();
    noInterrupts();
    InicioDoGiro=0; 
    pitch = 0;
	roll = 0;
	yaw = 0;
    settavaloresIniciaisParametros();
  }
  atualizaGiro();
  if(abs(roll)<abs(graus)){
    atualizaGiro();
    sendInt32(ANGULO_ATUAL, roll);
    if(graus<0)
      direitaEixo(VEL_GIRO_ESQ, VEL_GIRO_DIR);
    else
      esquerdaEixo(VEL_GIRO_ESQ, VEL_GIRO_DIR);
  }else{
    travar();
    interrupts(); 
    STATE = 1;
    InicioDoGiro = 1;
    settavaloresIniciaisParametros();
    sendInt32(ACABOU_GIRO,ACABOU_GIRO);
  }
}
