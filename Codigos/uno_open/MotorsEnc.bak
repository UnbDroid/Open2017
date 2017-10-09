#define ENCODER_LEFT 2
#define ENCODER_RIGHT 3
#define WEEL_DIAM 10
#define MOTOR_RIGHT 1
#define MOTOR_LEFT 2
#define MAL 6
#define MBL 4
#define MOTORL_VEL 5
#define MAR 9
#define MBR 10
#define MOTORR_VEL 11
#define POT 120

int allowEncoder, dir;
volatile long int encCountLeft[2], encCountRight[2], coord[2];

void OnFwd(int motor, int power){
  if(motor==MOTOR_LEFT){
    digitalWrite(MAL, 1);
    digitalWrite(MBL, 0);
    analogWrite(MOTORL_VEL, power);
  }
  else{
  
    if(motor==MOTOR_RIGHT){
      digitalWrite(MAR, 1);
      digitalWrite(MBR, 0);
      analogWrite(MOTORR_VEL, power);    
    }
    else{
      //manda mensagem de erro
    }
  }
}


void OffMotors(){
  digitalWrite(MAL, 1);
  digitalWrite(MBL, 1);
  analogWrite(MOTORL_VEL, 0);
  digitalWrite(MAR, 1);
  digitalWrite(MBR, 1);
  analogWrite(MOTORR_VEL, 0);
}
      
void OnRev(int motor, int power){
  if(motor==MOTOR_LEFT){
    digitalWrite(MAL, 0);
    digitalWrite(MBL, 1);
    analogWrite(MOTORL_VEL, power);
  }
  else{
    if(motor==MOTOR_RIGHT){
      digitalWrite(MAR, 0);
      digitalWrite(MBR, 1);
      analogWrite(MOTORR_VEL, power);
    }
    else{
      //manda mensagem de erro
    }
  }
}


float DegreeToCm(float dg){                              // Converte o ângulo da roda para centímetros percorridos por ela
  return (((dg * PI)/360) * WEEL_DIAM);
}

float Degree(long int count){                             // Converte a contagem do encoder para graus
  return ((count) * (360) / (16 * 131));
}

void AddEncoderLeft(){
  //Serial.println("inter1");
  if(allowEncoder){
    encCountLeft[abs(dir)-1]  +=  ((dir)/abs(dir));
  }
}

void AddEncoderRight(){
  //Serial.println("inter2");
  if(allowEncoder){
    encCountRight[abs(dir)-1]  +=  ((dir)/abs(dir));
  }
}

void SetupInterrupt(){
  encCountLeft[0]=0;
  encCountLeft[1]=0;
  encCountRight[0]=0;
  encCountRight[1]=0;
  coord[0]=0;
  coord[1]=0;
  allowEncoder=1;
  dir = 1;
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), AddEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), AddEncoderRight, RISING);
}
