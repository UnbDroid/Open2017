#ifndef MAMADOR_H
#define MAMADOR_H

#include <Arduino.h>

#define PRECISAODir 0.0225 //0.18
#define PRECISAOEsq 0.0243 //0.195
#define ORIGEMEsq 0
#define ORIGEMDir 0
#define FREQUENCIA 1000.0
#define POTENCIA 50

class MamadorOpen
{
  public:
    void iniciaDir (int pin_stp, int pin_direcao, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso);
    void iniciaEsq (int pin_stp, int pin_direcao, int pin_rst, int pin_slp, int pin_ena, int pin_m0, int pin_m1, int pin_m2, int fim_curso, int inicio_curso);
    void posicao(bool direcao);
    void ordenha(int potencia);
    void pinoMotor (int pin);

  private:
    int motor;
    int posDir, stpDir, dirDir, rstDir, slpDir, enaDir, m0Dir, m1Dir, m2Dir, passoAtualDir, fimCursoDir, inicioCursoDir;
    int posEsq, stpEsq, dirEsq, rstEsq, slpEsq, enaEsq, m0Esq, m1Esq, m2Esq, passoAtualEsq, fimCursoEsq, inicioCursoEsq;
};

#endif
