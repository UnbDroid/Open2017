#include "MamadorOpen.h"


#define ORIGEMDir 0 //Sentido rotação para o zero, vc definirá o que é zero
#define ORIGEMEsq 0 //Sentido rotação para o zero, vc definirá o que é zero



void MamadorOpen::iniciaEsq (int pin_stp, int pin_direcao, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso)
{
  stpEsq = pin_stp; //Pino de Passo
  dirEsq = pin_direcao; //Pino de Direção
  rstEsq = pin_rst; //Pino de Resetar
  slpEsq = pin_slp; //Pino de função Sleep
  enaEsq = pin_ena; //Pino de Enable
  m0Esq = pin_m0; //Configura full-step, half-step, etc
  m1Esq = pin_m1; //Configura full-step, half-step, etc
  m2Esq = pin_m2; //Configura full-step, half-step, etc
  fimCursoEsq = fim_curso;      //Pino fim de curso
  inicioCursoEsq = inicio_curso;//Pino inicio de curso

  pinMode (stpEsq, OUTPUT);  pinMode (dirEsq, OUTPUT);  pinMode (rstEsq, OUTPUT);  pinMode (slpEsq, OUTPUT);       pinMode (enaEsq, OUTPUT);
  pinMode (m0Esq, OUTPUT);   pinMode (m1Esq, OUTPUT);   pinMode (m2Esq, OUTPUT);   pinMode (fimCursoEsq, INPUT_PULLUP);  pinMode (inicioCursoEsq, INPUT_PULLUP);

  //Processo de inciação do driver
  digitalWrite(enaEsq, HIGH);  // Desativa o chip DRV8825
  delay (10);                // Atraso de 10 milisegundos
  digitalWrite(slpEsq, HIGH);  // Desativa modo sleep do DRV8825
  digitalWrite(rstEsq, LOW);   // Realiza o reset do DRV8825
  delay (1);                 // Atraso de 1 milisegundo
  digitalWrite(rstEsq, HIGH);  // Libera o reset do DRV8825
  delay (10);                // Atraso de 10 milisegundos
  digitalWrite(enaEsq, LOW);   // Ativa as saidas DRV8825

  // Configura modo 1/8 (1/8 step)
  digitalWrite(m0Esq, HIGH);
  digitalWrite(m1Esq, HIGH);
  digitalWrite(m2Esq, LOW);
}

void MamadorOpen::iniciaDir (int pin_stp, int pin_direcao, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso)
{
  stpDir = pin_stp; //Pino de Passo
  dirDir = pin_direcao; //Pino de Direção
  rstDir = pin_rst; //Pino de Resetar
  slpDir = pin_slp; //Pino de função Sleep
  enaDir = pin_ena; //Pino de Enable
  m0Dir = pin_m0;   //Configura full-step, half-step, etc
  m1Dir = pin_m1;   //Configura full-step, half-step, etc
  m2Dir = pin_m2;   //Configura full-step, half-step, etc
  fimCursoDir = fim_curso;      //Pino fim de curso
  inicioCursoDir = inicio_curso;//Pino inicio de curso

  pinMode (stpDir, OUTPUT);  pinMode (dirDir, OUTPUT);  pinMode (rstDir, OUTPUT);  pinMode (slpDir, OUTPUT);        pinMode (enaDir, OUTPUT);
  pinMode (m0Dir, OUTPUT);   pinMode (m1Dir, OUTPUT);   pinMode (m2Dir, OUTPUT);   pinMode (fimCursoDir, INPUT_PULLUP); pinMode (inicioCursoDir, INPUT_PULLUP);

  //Processo de inciação do driver
  digitalWrite(enaDir, HIGH);  // Desativa o chip DRV8825
  delay (10);                // Atraso de 10 milisegundos
  digitalWrite(slpDir, HIGH);  // Desativa modo sleep do DRV8825
  digitalWrite(rstDir, LOW);   // Realiza o reset do DRV8825
  delay (1);                 // Atraso de 1 milisegundo
  digitalWrite(rstDir, HIGH);  // Libera o reset do DRV8825
  delay (10);                // Atraso de 10 milisegundos
  digitalWrite(enaDir, LOW);   // Ativa as saidas DRV8825

  // Configura modo 1/8 (1/8 step)
  digitalWrite(m0Dir, HIGH);
  digitalWrite(m1Dir, HIGH);
  digitalWrite(m2Dir, LOW);
}
void MamadorOpen::pinoMotor (int pin)
{
  motor = pin;
  pinMode (motor, OUTPUT);
}
void MamadorOpen::ordenha (int potencia){
  analogWrite (motor, potencia);
}
void MamadorOpen::posicao(bool direcao)
{
  int sensorEsq, sensorDir;
  if (direcao){
    digitalWrite(dirDir, ORIGEMDir);
    digitalWrite(dirEsq, ORIGEMEsq);
    sensorEsq = inicioCursoEsq;
    sensorDir = inicioCursoDir;
  }
  else {
    digitalWrite(dirDir, !ORIGEMDir);
    digitalWrite(dirEsq, !ORIGEMEsq);
    sensorEsq = fimCursoEsq;
    sensorDir = fimCursoDir;
  }
  //Acelera
  int periodo;
  bool esqZerado = HIGH, dirZerado = HIGH;
  for (int i = 1; i <= 10; i++) {
    if (!digitalRead (sensorEsq)) {
      esqZerado = LOW;
    }
    if (!digitalRead (sensorDir)) {
      dirZerado = LOW;
    }
    periodo =  round (FREQUENCIA * 15 / i);
    digitalWrite(stpEsq, LOW);
    digitalWrite(stpDir, LOW);
    delayMicroseconds (periodo);
    digitalWrite(stpDir, dirZerado);
    digitalWrite(stpEsq, esqZerado);
    delayMicroseconds (periodo);
  }
  //Mantem velocidade
  while (esqZerado&&dirZerado)
  {
    if (!digitalRead (sensorEsq)) {
      esqZerado = LOW;
    }
    if (!digitalRead (sensorDir)){
      dirZerado = LOW;
    }
    digitalWrite(stpEsq, LOW);
    digitalWrite(stpDir, LOW);
    delayMicroseconds (FREQUENCIA * 1.5);

    digitalWrite(stpEsq, esqZerado);
    digitalWrite(stpDir, dirZerado);
    delayMicroseconds (FREQUENCIA * 1.5);
  }
}

