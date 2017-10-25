#include "GarraOpen.h"


//INICIAÇÃO DA GARRA ---------------------------------------------

void GarraOpen::iniciaServos(int pinCot, int pinPul, int pinAtu)
{
  cotovelo.attach(pinCot);
  pulso.attach(pinPul);
  atuador.attach(pinAtu);
}

void GarraOpen::iniciaX (int pin_stp, int pin_dir, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso) {
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
  pinMode (m0X, OUTPUT);   pinMode (m1X, OUTPUT);   pinMode (m2X, OUTPUT);   pinMode (fimCursoX, OUTPUT);  pinMode (inicioCursoX, OUTPUT);

  //Processo de inciação do driver
  digitalWrite(enaX, HIGH);  // Desativa o chip DRV8825
  delay (10);                // Atraso de 10 milisegundos
  digitalWrite(slpX, HIGH);  // Desativa modo sleep do DRV8825
  digitalWrite(rstX, LOW);   // Realiza o reset do DRV8825
  delay (1);                 // Atraso de 1 milisegundo
  digitalWrite(rstX, HIGH);  // Libera o reset do DRV8825
  delay (10);                // Atraso de 10 milisegundos
  digitalWrite(enaX, LOW);   // Ativa as saidas DRV8825

  // Configura modo Passo completo (Full step)
  digitalWrite(m0X, LOW);
  digitalWrite(m1X, LOW);
  digitalWrite(m2X, LOW);
}

void GarraOpen::iniciaY (int pin_stp, int pin_dir, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso) {
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
  pinMode (m0Y, OUTPUT);   pinMode (m1Y, OUTPUT);   pinMode (m2Y, OUTPUT);   pinMode (fimCursoY, OUTPUT);  pinMode (inicioCursoY, OUTPUT);

  //Processo de inciação do driver
  digitalWrite(enaY, HIGH);  // Desativa o chip DRV8825
  delay (10);                // Atraso de 10 milisegundos
  digitalWrite(slpY, HIGH);  // Desativa modo sleep do DRV8825
  digitalWrite(rstY, LOW);   // Realiza o reset do DRV8825
  delay (1);                 // Atraso de 1 milisegundo
  digitalWrite(rstY, HIGH);  // Libera o reset do DRV8825
  delay (10);                // Atraso de 10 milisegundos
  digitalWrite(enaY, LOW);   // Ativa as saidas DRV8825

  // Configura modo Passo completo (Full step)
  digitalWrite(m0Y, LOW);
  digitalWrite(m1Y, LOW);
  digitalWrite(m2Y, LOW);
}

void GarraOpen::zeraGarra(bool novaPos)
{
  digitalWrite(dirY, ORIGEMY);
  digitalWrite(dirX, ORIGEMX);

  //Acelera
  int periodo;
  for (int i = 1; i <= 10; i++) {
    periodo = FREQUENCIA * 20 / i;
    digitalWrite(stpX, LOW);
    digitalWrite(stpY, LOW);
    delayMicroseconds (periodo);
    digitalWrite(stpY, HIGH);
    digitalWrite(stpX, HIGH);
    delayMicroseconds (periodo);
  }
  //Mantem velocidade
  bool xZerado = HIGH, yZerado = HIGH;
  while (digitalRead (inicioCursoY)) 
  {  
    digitalWrite(stpX, LOW);     
    digitalWrite(stpY, LOW);     
    delayMicroseconds (FREQUENCIA * 2);
     
    digitalWrite(stpX, xZerado);    
    digitalWrite(stpY, yZerado);    
    delayMicroseconds (FREQUENCIA * 2);
    
    if (digitalRead (inicioCursoX))
      xZerado = LOW;
    if (digitalRead (inicioCursoY))
      yZerado = LOW;
  }
  passoAtualX = 0;
  passoAtualY = 0;

  pulso.write (PULSO_CIMA);
  atuador.write (ATUADOR_ABRE);
  cotovelo.write (COTOVELO_FRENTE);
  
  posPulso = false;
  posAtuador = false;
  posCotovelo = false;
}

//CONTROLE DOS SERVOS DA GARRA ---------------------------------------------
void GarraOpen::movePulso(bool novaPos)
{
  if (novaPos ^ posPulso) //Xor(novaPos, posPulso)
  {
    if (novaPos)
      pulso.write (PULSO_BAIXO);
    else
      pulso.write (PULSO_CIMA);
      
  }
}

void GarraOpen::moveAtuador(bool novaPos)
{
  if (novaPos ^ posAtuador) //Xor(novaPos, posAtuador)
  {
    if (novaPos)
      atuador.write (ATUADOR_FECHA);
    else
      atuador.write (ATUADOR_ABRE);
      
  }
}

void GarraOpen::moveCotovelo(bool novaPos)
{
  if (novaPos ^ posCotovelo) //Xor(novaPos, posCotovelo)
  {
    if (novaPos)
      cotovelo.write (COTOVELO_TRAS);
    else
      cotovelo.write (COTOVELO_FRENTE);
  }
}



//CONTROLE DOS MOTORES DE PASSO DA GARRA --------------------------------------------

void GarraOpen::moveX (int novaPos) {
  int qtdPasso = novaPos / PRECISAOX;
  qtdPasso -= passoAtualX;

  bool dir = qtdPasso > 0 ? 0 : 1;
  digitalWrite(dirX, dir);

  int periodo;
  for (int i = 1; i <= 1 + qtdPasso / 2; i++) {
    periodo = FREQUENCIA * qtdPasso / i;
    digitalWrite(stpX, LOW);     // Pulso nível baixo
    delayMicroseconds (periodo);   // MeioPeriodo de X microsegundos
    digitalWrite(stpX, HIGH);    // Pulso nível alto
    delayMicroseconds (periodo);
  }
  for (int i = 1; i <= 1 + qtdPasso / 2; i++) {
    periodo = FREQUENCIA * i;
    digitalWrite(stpX, LOW);            // Pulso nível baixo
    delayMicroseconds (periodo);   // MeioPeriodo de X microsegundos
    digitalWrite(stpX, HIGH);           // Pulso nível alto
    delayMicroseconds (periodo);
  }
}

void GarraOpen::moveY (int novaPos) {
  int qtdPasso = novaPos / PRECISAOY;
  qtdPasso -= passoAtualY;

  bool dir = qtdPasso > 0 ? 0 : 1;
  digitalWrite(dirY, dir);

  int periodo;
  for (int i = 1; i <= 1 + qtdPasso / 2; i++) {
    periodo = FREQUENCIA * qtdPasso / i;
    digitalWrite(stpY, LOW);            // Pulso nível baixo
    delayMicroseconds (periodo);   // MeioPeriodo de X microsegundos
    digitalWrite(stpY, HIGH);           // Pulso nível alto
    delayMicroseconds (periodo);
  }
  for (int i = 1; i <= 1 + qtdPasso / 2; i++) {
    periodo = FREQUENCIA * i;
    digitalWrite(stpY, LOW);            // Pulso nível baixo
    delayMicroseconds (periodo);   // MeioPeriodo de X microsegundos
    digitalWrite(stpY, HIGH);           // Pulso nível alto
    delayMicroseconds (periodo);
  }
}
