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

#define PRECISAOX 0.0225 //0.18
#define PRECISAOY 0.0243 //0.195
#define ORIGEMY 0
#define ORIGEMX 0
#define FREQUENCIA 300.0

class GarraOpen
{
  public:
    void iniciaServos (int pinCot, int pinPul, int pinAtu);
    void iniciaX (int pin_stp, int pin_dir, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso);
    void iniciaY (int pin_stp, int pin_dir, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso);

    void zeraGarra();
    
    void movePulso(bool pos);
    void moveAtuador(bool pos);
    void moveCotovelo(bool pos);

    void passoX();
    void passoY();
    void moveX (int pos);
    void moveY (int pos);
   

  private:
    Servo cotovelo, pulso, atuador;
    int posX, stpX, dirX, rstX, slpX, enaX, m0X, m1X, m2X, passoAtualX, fimCursoX, inicioCursoX;
    int posY, stpY, dirY, rstY, slpY, enaY, m0Y, m1Y, m2Y, passoAtualY, fimCursoY, inicioCursoY;
    bool posPulso, posAtuador, posCotovelo;
};

#endif
