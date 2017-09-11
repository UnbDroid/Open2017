#define ENCODER_LEFT 0
#define ENCODER_RIGHT 0
#define WEEL_DIAM 0
#define MOTOR_RIGHT 0
#define MOTOR_LEFT 0
#define MAL 0
#define MBL 0
#define MAR 0
#define MBR 0
#define MOTORR_VEL 0
#define MOTORL_VEL 0


int allowEncoder,dir;
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


void OffMotors(){
  digitalWrite(MAL, 1);
  digitalWrite(MBL, 1);
  analogWrite(MOTORL_VEL, 0);
  digitalWrite(MAR, 1);
  digitalWrite(MBR, 1);
  analogWrite(MOTORR_VEL, 0);
}


float DegreeToCm(float dg){                              // Converte o ângulo da roda para centímetros percorridos por ela
  return (((dg * PI)/360) * WEEL_DIAM);
}

float Degree(long int count){                             // Converte a contagem do encoder para graus
  return ((count) * (360) / (16 * 131));
}

void AddEncoderLeft(){
  if(allowEncoder){
    encCountLeft[abs(dir)-1]  +=  ((dir)/abs(dir));
  }
}

void AddEncoderRight(){
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
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), AddEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), AddEncoderRight, RISING);
}
