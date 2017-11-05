#ifndef GARRA_H
#define GARRA_H

#include <Arduino.h>
#include "Servo.h"

#define RED A0
#define BLUE A1
#define GREEN A2

#define FIM_CURSO_X 0
#define FIM_CURSO_Y 0

#define INICIO_CURSO_X 0
#define INICIO_CURSO_Y 0

#define PULSO_CIMA 10
#define PULSO_BAIXO 165

#define ATUADOR_ABRE 80
#define ATUADOR_FECHA 2

#define COTOVELO_FRENTE 10
#define COTOVELO_TRAS 168

#define PRECISAOX 0.0225 //0.18
#define PRECISAOY 0.0243 //0.195
#define ORIGEMY 0
#define ORIGEMX 0
#define FREQUENCIA 1000.0

#define QUANTIDADE_MOTORES_GARRA 5

#define MOTOR_PASSO_X 0
#define MOTOR_PASSO_Y 1
#define MOTOR_SERVO_COTOVELO 2
#define MOTOR_SERVO_PULSO 3
#define MOTOR_SERVO_ATUADOR 4

#define ACABOU_GARRA 5

#define NUM_IDEN_GARRA 600


class GarraOpen
{
  public:
    void iniciaServos (int pinCot, int pinPul, int pinAtu);
    void iniciaX (int pin_stp, int pin_dir, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso);
    void iniciaY (int pin_stp, int pin_dir, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso);

    
    int trataGarra(int id, long long int data);
    bool ehGarra(int id);
    
    
    void zeraGarra();
    void setupGarra();
    
    bool movePulso(bool pos);
    bool moveAtuador(bool pos);
    bool moveCotovelo(bool pos);

    void passoX();
    void passoY();
    bool moveX (int pos);
    bool moveY (int pos);

    void seVira (int posx, int posy, bool serCotovelo, bool serPulso, bool serAtuador);
  private:
    Servo cotovelo, pulso, atuador;
    int posX, stpX, dirX, rstX, slpX, enaX, m0X, m1X, m2X, passoAtualX, fimCursoX, inicioCursoX;
    int posY, stpY, dirY, rstY, slpY, enaY, m0Y, m1Y, m2Y, passoAtualY, fimCursoY, inicioCursoY;
    bool posPulso, posAtuador, posCotovelo;
};

#endif
