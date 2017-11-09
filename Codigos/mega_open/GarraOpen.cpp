#include "GarraOpen.h"

//INICIAÇÃO DA GARRA ---------------------------------------------


bool GarraOpen::ehGarra(int id)
{
  id = id - NUM_IDEN_GARRA;
  if (id < QUANTIDADE_MOTORES_GARRA && id >= 0)  return true;
  else  return false;
}

int GarraOpen::trataGarra(int id, long long int data) {
  switch (id - NUM_IDEN_GARRA)
  {
    case MOTOR_PASSO_X:
      //return (MOTOR_PASSO_X+NUM_IDEN_GARRA)*GarraOpen::moveX(data);
      
    case MOTOR_PASSO_Y:
      //return (MOTOR_PASSO_Y+NUM_IDEN_GARRA)*GarraOpen::moveY(data);
      
    case MOTOR_SERVO_COTOVELO:
      //return (MOTOR_SERVO_COTOVELO+NUM_IDEN_GARRA)*GarraOpen::moveCotovelo((data >= 1));
      
    case MOTOR_SERVO_PULSO:
      //return (MOTOR_SERVO_PULSO+NUM_IDEN_GARRA)*GarraOpen::movePulso((data >= 1));
      
    case MOTOR_SERVO_ATUADOR:
      //return (MOTOR_SERVO_ATUADOR+NUM_IDEN_GARRA)*GarraOpen::moveAtuador((data >= 1));    
    default:    return -1;
  }
}
void GarraOpen::iniciaServos(int pinCot, int pinPul, int pinAtu)
{
  cotovelo.attach(pinCot);
  pulso.attach(pinPul);
  atuador.attach(pinAtu);

  cotovelo.write (COTOVELO_FRENTE);
  pulso.write (PULSO_CIMA);
  atuador.write (ATUADOR_ABRE);

  posPulso = PULSO_CIMA;
  posAtuador = ATUADOR_ABRE;
  posCotovelo = COTOVELO_FRENTE;
}
 void GarraOpen::setupGarra()
 {
  this->iniciaY (18, 17, 20, 19, 11, 12, 13, 21, 42, 11);
  this->iniciaX (5, 6, 3, 4, 16, 15, 14, 2, 44, 40);
  this->zeraGarra ();
  //this->.iniciaServos(A1, A2, A0);
}

void GarraOpen::iniciaX (int pin_stp, int pin_dir, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso)
{
 stpX = pin_stp; //Pino de Passo
  dirX = pin_dir; //Pino de Direção
  rstX = pin_rst; //Pino de Resetar
  slpX = pin_slp; //Pino de função Sleep
  enaX = pin_ena; //Pino de Enable
  m0X = pin_m0; //Configura full-step, half-step, etc
  m1X = pin_m1; //Configura full-step, half-step, etc
  m2X = pin_m2; //Configura full-step, half-step, etc
  fimCursoX = fim_curso;      //Pino fim de curso
  inicioCursoX = inicio_curso;//Pino inicio de curso

  pinMode (stpX, OUTPUT);  pinMode (dirX, OUTPUT);  pinMode (rstX, OUTPUT);  pinMode (slpX, OUTPUT);       pinMode (enaX, OUTPUT);
  pinMode (m0X, OUTPUT);   pinMode (m1X, OUTPUT);   pinMode (m2X, OUTPUT);   pinMode (fimCursoX, INPUT_PULLUP);  pinMode (inicioCursoX, INPUT_PULLUP);

  //Processo de inciação do driver
  digitalWrite(enaX, HIGH);  // Desativa o chip DRV8825
  delay (10);                // Atraso de 10 milisegundos
  digitalWrite(slpX, HIGH);  // Desativa modo sleep do DRV8825
  digitalWrite(rstX, LOW);   // Realiza o reset do DRV8825
  delay (1);                 // Atraso de 1 milisegundo
  digitalWrite(rstX, HIGH);  // Libera o reset do DRV8825
  delay (10);                // Atraso de 10 milisegundos
  digitalWrite(enaX, LOW);   // Ativa as saidas DRV8825

  // Configura modo 1/8 (1/8 step)
  digitalWrite(m0X, HIGH);
  digitalWrite(m1X, HIGH);
  digitalWrite(m2X, LOW);
}

void GarraOpen::iniciaY (int pin_stp, int pin_dir, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso)
{
  stpY = pin_stp; //Pino de Passo
  dirY = pin_dir; //Pino de Direção
  rstY = pin_rst; //Pino de Resetar
  slpY = pin_slp; //Pino de função Sleep
  enaY = pin_ena; //Pino de Enable
  m0Y = pin_m0;   //Configura full-step, half-step, etc
  m1Y = pin_m1;   //Configura full-step, half-step, etc
  m2Y = pin_m2;   //Configura full-step, half-step, etc
  fimCursoY = fim_curso;      //Pino fim de curso
  inicioCursoY = inicio_curso;//Pino inicio de curso

  pinMode (stpY, OUTPUT);  pinMode (dirY, OUTPUT);  pinMode (rstY, OUTPUT);  pinMode (slpY, OUTPUT);        pinMode (enaY, OUTPUT);
  pinMode (m0Y, OUTPUT);   pinMode (m1Y, OUTPUT);   pinMode (m2Y, OUTPUT);   pinMode (fimCursoY, INPUT_PULLUP); pinMode (inicioCursoY, INPUT_PULLUP);

  //Processo de inciação do driver
  digitalWrite(enaY, HIGH);  // Desativa o chip DRV8825
  delay (10);                // Atraso de 10 milisegundos
  digitalWrite(slpY, HIGH);  // Desativa modo sleep do DRV8825
  digitalWrite(rstY, LOW);   // Realiza o reset do DRV8825
  delay (1);                 // Atraso de 1 milisegundo
  digitalWrite(rstY, HIGH);  // Libera o reset do DRV8825
  delay (10);                // Atraso de 10 milisegundos
  digitalWrite(enaY, LOW);   // Ativa as saidas DRV8825

  // Configura modo 1/8 (1/8 step)
  digitalWrite(m0Y, HIGH);
  digitalWrite(m1Y, HIGH);
  digitalWrite(m2Y, LOW);
}

void GarraOpen::zeraGarra()
{
  digitalWrite(dirY, ORIGEMY); 
  digitalWrite(dirX, ORIGEMX);

  //Acelera
  int periodo;
  bool xZerado = HIGH, yZerado = HIGH;
  for (int i = 1; i <= 10; i++) {
    if (!digitalRead (inicioCursoX)) {
      xZerado = LOW;
      Serial.println ("x tocou");
    }
    if (!digitalRead (inicioCursoY)) {
      yZerado = LOW;
      Serial.println ("y tocou");
    }
    periodo =  round (FREQUENCIA * 15 / i);

    digitalWrite(stpX, LOW);
    digitalWrite(stpY, LOW);
    delayMicroseconds (periodo);

    digitalWrite(stpY, yZerado);
    digitalWrite(stpX, xZerado);
    delayMicroseconds (periodo);
  }
  //Mantem velocidade
  while /*(xZerado)/*/(xZerado || yZerado)
  {
    if (!digitalRead (inicioCursoX)) {
      xZerado = LOW;
      //Serial.println ("x tocou");
    }
    if (!digitalRead (inicioCursoY)) {
      yZerado = LOW;
      //Serial.println ("y tocou");
    }

    digitalWrite(stpX, LOW);
    digitalWrite(stpY, LOW);
    delayMicroseconds (FREQUENCIA * 1.5);

    digitalWrite(stpX, xZerado);
    digitalWrite(stpY, yZerado);
    delayMicroseconds (FREQUENCIA * 1.5);
  }
  passoAtualX = 0;
  passoAtualY = 0;
}

//CONTROLE DOS SERVOS DA GARRA ---------------------------------------------
void GarraOpen::movePulso(int novaPos)
{
  if (novaPos > posPulso)
  {
    for (int i = posPulso; i < novaPos; i++) {
      pulso.write(i);
      delay (30);
    }
  }
  else
  {
    for (int i = posPulso; i > novaPos; i--) {
      pulso.write(i);
      delay (30);
    }
  }
  posPulso = novaPos;
}

void GarraOpen::moveAtuador(int novaPos)
{
  if (novaPos > posAtuador)
  {
    for (int i = posAtuador; i < novaPos; i++) {
      atuador.write(i);
      delay (30);
    }
  }
  else
  {
    for (int i = posAtuador; i > novaPos; i--) {
      atuador.write(i);
      delay (30);
    }
  }
  posAtuador = novaPos;
}

void GarraOpen::moveCotovelo(int novaPos)
{
  if (novaPos > posCotovelo)
  {
    for (int i = posCotovelo; i < novaPos; i++) {
      cotovelo.write(i);
      delay (30);
    }
  }
  else
  {
    for (int i = posCotovelo; i > novaPos; i--) {
      cotovelo.write(i);
      delay (30);
    }
  }
  posCotovelo = novaPos;
}



//CONTROLE DOS MOTORES DE PASSO DA GARRA --------------------------------------------
void GarraOpen::passoX()
{
  digitalWrite(dirX, !ORIGEMX);
  for (int i = 0; i < 3500; i++) {
    digitalWrite(stpX, LOW);     // Pulso nível baixo
    delayMicroseconds (FREQUENCIA);   // MeioPeriodo de X microsegundos
    digitalWrite(stpX, HIGH);    // Pulso nível alto
    delayMicroseconds (FREQUENCIA);
  }
}


void GarraOpen::passoY()
{
  digitalWrite(dirY, !ORIGEMY);
  for (int i = 0; i < 3500; i++) {
    digitalWrite(dirY, !ORIGEMX);
    digitalWrite(stpY, LOW);     // Pulso nível baixo
    delayMicroseconds (FREQUENCIA);   // MeioPeriodo de X microsegundos
    digitalWrite(stpY, HIGH);    // Pulso nível alto
    delayMicroseconds (FREQUENCIA);
  }
}
void GarraOpen::moveX (int novaPos)
{
  int qtdPasso = novaPos / PRECISAOX;
  qtdPasso -= passoAtualX;
  int tempPasso = novaPos / PRECISAOX;
  passoAtualX = novaPos / PRECISAOX;

  bool dir;
  if (qtdPasso > 0) { //Se movimento positivo
    dir = 1;
  }
  else {  //Se movimento negativo
    dir = 0;
    qtdPasso = (-1) * qtdPasso;
  }
  digitalWrite (dirX, dir);
  unsigned long int periodo;
  unsigned long int temp = FREQUENCIA * qtdPasso;

  //ACELERAÇÃO----------------------
  for (int i = 1; i <= 1 + qtdPasso / 5; i++) {
    if (!(digitalRead (inicioCursoX) || !digitalRead (fimCursoX))) { //Se chegou a um limite
      if (dir != ORIGEMX && !digitalRead (fimCursoX)) { //Se movimento era positivo
        //Serial.println("fim");
        passoAtualX =  tempPasso + i; //Adiciona quantidade andada
        break;  //Para movimentação
      }
      else if (dir == ORIGEMX  && !(digitalRead (inicioCursoX))) {  //Se movimento era negativo
        passoAtualX =  0; //Subtratai quantidade andada
        //Serial.println("inicioCursoX");
        break;  //Para movimentação
      }
    }
    periodo = round (temp / (i * 5)); //Acelera
    //Serial.println(periodo, DEC);
    digitalWrite(stpX, LOW);     // Pulso nível baixo
    delayMicroseconds (periodo); // MeioPeriodo de X microsegundos
    digitalWrite(stpX, HIGH);    // Pulso nível alto
    delayMicroseconds (periodo); // MeioPeriodo de X microsegundos
  }

  //VELOCIDADE DE CRUZEIRO-------------
  for (int i = 1; i <= 1 + 3 * qtdPasso / 5; i++) {
    if (i > 1) {
      if (!(digitalRead (inicioCursoX) || !digitalRead (fimCursoX))) { //Se chegou a um limite
        if (dir != ORIGEMX && !digitalRead (fimCursoX)) { //Se movimento era positivo
          passoAtualX =  tempPasso + i; //Adiciona quantidade andada
          //Serial.println("fim");
          break;  //Para movimentação
        }
        else if (dir == ORIGEMX  && !(digitalRead (inicioCursoX))) {  //Se movimento era negativo
          passoAtualX =  0; //Subtratai quantidade andada
          //Serial.println("inicioCursoX");
          break;  //Para movimentação
        }
      }
    }
    digitalWrite(stpX, LOW);        // Pulso nível baixo
    delayMicroseconds (FREQUENCIA); // MeioPeriodo de X microsegundos
    digitalWrite(stpX, HIGH);       // Pulso nível alto
    delayMicroseconds (FREQUENCIA); // MeioPeriodo de X microsegundos
  }

  //DESACELERAÇÃO---------------------
  for (int i = 1; i <= 1 + qtdPasso / 5; i++) {
    if (i > 1) {  //Se nao foi chegado ao fim do curso na aceleração
      if (!(digitalRead (inicioCursoX) || !digitalRead (fimCursoX))) { //Se chegou a um limite
        if (dir != ORIGEMX && !digitalRead (fimCursoX)) { //Se movimento era positivo
          passoAtualX =  tempPasso + i; //Adiciona quantidade andada
          break;  //Para movimentação
        }
        else if (dir == ORIGEMX  && !(digitalRead (inicioCursoX))) {  //Se movimento era negativo
          passoAtualX =  0; //Subtratai quantidade andada
          break;  //Para movimentação
        }
      }
    }
    periodo = FREQUENCIA * i;     //Desacelera
    //Serial.println(periodo, DEC);
    digitalWrite(stpX, LOW);      // Pulso nível baixo
    delayMicroseconds (periodo);  // MeioPeriodo de X microsegundos
    digitalWrite(stpX, HIGH);     // Pulso nível alto
    delayMicroseconds (periodo);  // MeioPeriodo de X microsegundos
  }
}

void GarraOpen::moveY (int novaPos) {
  int qtdPasso = novaPos / PRECISAOY;
  qtdPasso -= passoAtualY;
  int tempPasso = novaPos / PRECISAOY;
  passoAtualY = novaPos / PRECISAOY;

  bool dir;
  if (qtdPasso > 0) { //Se movimento positivo
    dir = 1;
  }
  else {  //Se movimento negativo
    dir = 0;
    qtdPasso = (-1) * qtdPasso;
  }

  digitalWrite (dirY, dir);

  unsigned long int periodo;
  unsigned long int temp = FREQUENCIA * qtdPasso;

  //ACELERAÇÃO----------------------
  for (int i = 1; i <= 1 + qtdPasso / 5; i++) {
    if (!(digitalRead (inicioCursoY) || !digitalRead (fimCursoY))) { //Se chegou a um limite
      if (dir != ORIGEMY && digitalRead (fimCursoY)) { //Se movimento era positivo
        passoAtualY =  tempPasso + i; //Adiciona quantidade andada
        break;  //Para movimentação
      }
      else if (dir == ORIGEMY && !(digitalRead (inicioCursoY))) {  //Se movimento era negativo
        passoAtualY =  0; //Subtratai quantidade andada
        break;  //Para movimentação
      }
    }
    periodo = round (temp / (i * 5)); //Acelera
    //Serial.println(periodo, DEC);
    digitalWrite(stpY, LOW);     // Pulso nível baixo
    delayMicroseconds (periodo); // MeioPeriodo de X microsegundos
    digitalWrite(stpY, HIGH);    // Pulso nível alto
    delayMicroseconds (periodo); // MeioPeriodo de X microsegundos
  }

  //VELOCIDADE DE CRUZEIRO-------------
  for (int i = 1; i <= 1 + 3 * qtdPasso / 5; i++) {
    if (i > 1) {
      if (!(digitalRead (inicioCursoY) || !digitalRead (fimCursoY))) { //Se chegou a um limite
        if (dir != ORIGEMY && digitalRead (fimCursoY)) { //Se movimento era positivo
          passoAtualY =  tempPasso + i; //Adiciona quantidade andada
          break;  //Para movimentação
        }
        else if (dir == ORIGEMY && !(digitalRead (inicioCursoY))) {  //Se movimento era negativo
          passoAtualY =  0; //Subtratai quantidade andada
          break;  //Para movimentação
        }
      }
    }
    digitalWrite(stpY, LOW);        // Pulso nível baixo
    delayMicroseconds (FREQUENCIA); // MeioPeriodo de X microsegundos
    digitalWrite(stpY, HIGH);       // Pulso nível alto
    delayMicroseconds (FREQUENCIA); // MeioPeriodo de X microsegundos
  }

  //DESACELERAÇÃO---------------------
  for (int i = 1; i <= 1 + qtdPasso / 5; i++) {
    if (i > 1) {  //Se nao foi chegado ao fim do curso na aceleração
      if (!(digitalRead (inicioCursoY) || !digitalRead (fimCursoY))) { //Se chegou a um limite
        if (dir != ORIGEMY && digitalRead (fimCursoY)) { //Se movimento era positivo
          passoAtualY =  tempPasso + i; //Adiciona quantidade andada
          break;  //Para movimentação
        }
        else if (dir == ORIGEMY && !(digitalRead (inicioCursoY))) {  //Se movimento era negativo
          passoAtualY =  0; //Subtratai quantidade andada
          break;  //Para movimentação
        }
      }
    }
    periodo = FREQUENCIA * i;     //Desacelera
    //Serial.println(periodo, DEC);
    digitalWrite(stpY, LOW);      // Pulso nível baixo
    delayMicroseconds (periodo);  // MeioPeriodo de X microsegundos
    digitalWrite(stpY, HIGH);     // Pulso nível alto
    delayMicroseconds (periodo);  // MeioPeriodo de X microsegundos
  }
}

void GarraOpen::segueTrajetoria (bool dir) {

  int posicaoCritica[2][19] = {{3, 2, 0, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2},
    {ATUADOR_FECHA, COTOVELO_FRENTE, 0, 45, 50, 50, 45, 60, 43, 67, 38, 81, 37, 91, 27, 95, 24, 105, COTOVELO_TRAS}
  };
  if (dir == 0) {
    for (int i = 0; i < 19; i++) {
      switch (posicaoCritica[0][i]) {
        case 0:
          this->moveX(posicaoCritica[1][i]);
          break;
        case 1:
          this->moveY(posicaoCritica[1][i]);
          break;
        case 2:
          this->moveCotovelo(posicaoCritica[1][i]);
          break;
        case 3:
          this->moveAtuador(posicaoCritica[1][i]);
          break;
        default:
          break;
      }
    }
  }
  else {
    for (int i = 18; i >= 0; i--) {
      switch (posicaoCritica[0][i]) {
        case 0:
          this->moveX(posicaoCritica[1][i]);
          break;
        case 1:
          this->moveY(posicaoCritica[1][i]);
          break;
        case 2:
          this->moveCotovelo(posicaoCritica[1][i]);
          break;
        case 3:
          this->moveAtuador(posicaoCritica[1][i]);
          break;
        default:
          break;
      }
    }
  }
}


    

