#ifndef GARRA_H
#define GARRA_H

#include <Arduino.h>
#include "Servo.h"


#define PULSO_CIMA 12
#define PULSO_BAIXO 167

#define ATUADOR_ABRE 75
#define ATUADOR_FECHA 10

#define COTOVELO_FRENTE 170
#define COTOVELO_TRAS 5

#define PRECISAOX 0.022571429 //0.18
#define PRECISAOY 0.02485714 //0.195
#define ORIGEMY 0
#define ORIGEMX 0
#define FREQUENCIA 1000.0

class GarraOpen
{
  public:
    void iniciaServos (int pinCot, int pinPul, int pinAtu);
    void iniciaX (int pin_stp, int pin_dir, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso);
    void iniciaY (int pin_stp, int pin_dir, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso);

    void zeraGarra();

    void movePulso(int pos);
    void moveAtuador(int pos);
    void moveCotovelo(int pos);

    void passoX();
    void passoY();
    void moveX (int pos);
    void moveY (int pos);

    void seVira (int posx, int posy, bool serCotovelo, bool serPulso, bool serAtuador);
    void segueTrajetoria (bool dir);
  
  private:
    Servo cotovelo, pulso, atuador;
    int posX, stpX, dirX, rstX, slpX, enaX, m0X, m1X, m2X, passoAtualX, fimCursoX, inicioCursoX;
    int posY, stpY, dirY, rstY, slpY, enaY, m0Y, m1Y, m2Y, passoAtualY, fimCursoY, inicioCursoY;
    int posPulso, posAtuador, posCotovelo;
};

#endif
