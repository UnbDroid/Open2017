#ifndef GARRA_H
#define GARRA_H

#include <Arduino.h>
#include <Servo.h>

#define PULSO_CIMA 10
#define PULSO_BAIXO 165

#define ATUADOR_ABRE 5
#define ATUADOR_FECHA 110

#define COTOVELO_FRENTE 10
#define COTOVELO_TRAS 168

#define PRECISAOX 2
#define PRECISAOY 2
#define FREQUENCIA 2000

class Garra
{
  public:
    Garra (int pinCot, int pinPul, int pinAtu, int pinDireX, int pinDireY);
    void movePulso(bool pos);
    void moveAtuador(bool pos);
    void moveCotovelo(bool pos);
    void moveX (int pos);
    void moveY (int pos);

  private:
    Servo cotovelo, pulso, atuador;
    int posX, posY, pinDirX, pinDirY, pinStpX, pinStpY;
    bool posPulso, posAtuador, posCotovelo;
};

#endif
