#include "GarraOpen.h"

GarraOpen::GarraOpen(int pinCot, int pinPul, int pinAtu, int pinDireX, int pinDireY, int pinStepX, int pinStepY)
{
  cotovelo.attach(pinCot);
  pulso.attach(pinPul);
  atuador.attach(pinAtu);

  pinStpX = pinStepX;
  pinStpY = pinStepY;
  pinDirX = pinDireX;
  pinDirY = pinDireY;
  posPulso = false;
  posAtuador = false;
  posCotovelo = false;
}

void GarraOpen::movePulso(bool novaPos)
{
  if (novaPos^posPulso){
    if (novaPos)
      pulso.write (PULSO_CIMA);
    else
      pulso.write (PULSO_BAIXO);
  }
}

void GarraOpen::moveAtuador(bool novaPos)
{
  if (novaPos^posAtuador){
    if (novaPos)
      pulso.write (ATUADOR_ABRE);
    else
      pulso.write (ATUADOR_FECHA);
  }
}

void GarraOpen::moveCotovelo(bool novaPos)
{
   if (novaPos^posCotovelo){
    if (novaPos)
      pulso.write (COTOVELO_FRENTE);
    else
      pulso.write (COTOVELO_TRAS);
  }
}

void passo(int pino, int delay)                    // Pulso do passo do Motor
{
  digitalWrite(pino, LOW);            // Pulso nível baixo
  delayMicroseconds (delay);   // MeioPeriodo de X microsegundos
  digitalWrite(pino, HIGH);           // Pulso nível alto
  delayMicroseconds (MeidelayoPeriodo);   // MeioPeriodo de X microsegundos
}

void GarraOpen::moveX (int novaPos){
  int qtdPasso = novaPos/PRECISAOX;
  qtdPasso -= passoAtual;

  bool dir = qtdPasso > 0?0:1
  digitalWrite(pinDirX, dir);

  int delay;
  for (int i = 1; i<= 1+qtdPasso/2; i++){
    delay = FREQUENCIA*qtdPasso/i;
    digitalWrite(pinDirX, LOW);            // Pulso nível baixo
    delayMicroseconds (delay);   // MeioPeriodo de X microsegundos
    digitalWrite(pinDirX, HIGH);           // Pulso nível alto
    delayMicroseconds (delay);
  }
  for (int i = 1; i<= 1+qtdPasso/2; i++){
    delay = FREQUENCIA*i;
    digitalWrite(pinDirX, LOW);            // Pulso nível baixo
    delayMicroseconds (delay);   // MeioPeriodo de X microsegundos
    digitalWrite(pinDirX, HIGH);           // Pulso nível alto
    delayMicroseconds (delay);
  }
}

void GarraOpen::moveY (int novaPos){
  int qtdPasso = novaPos/PRECISAOY;
  qtdPasso -= passoAtual;

  bool dir = qtdPasso > 0?0:1
  digitalWrite(pinDirY, dir);

  int delay;
  for (int i = 1; i<= 1+qtdPasso/2; i++){
    delay = FREQUENCIA*qtdPasso/i;
    digitalWrite(pinDirY, LOW);            // Pulso nível baixo
    delayMicroseconds (delay);   // MeioPeriodo de X microsegundos
    digitalWrite(pinDirY, HIGH);           // Pulso nível alto
    delayMicroseconds (delay);
  }
  for (int i = 1; i<= 1+qtdPasso/2; i++){
    delay = FREQUENCIA*i;
    digitalWrite(pinDirY, LOW);            // Pulso nível baixo
    delayMicroseconds (delay);   // MeioPeriodo de X microsegundos
    digitalWrite(pinDirY, HIGH);           // Pulso nível alto
    delayMicroseconds (delay);
  }
}
