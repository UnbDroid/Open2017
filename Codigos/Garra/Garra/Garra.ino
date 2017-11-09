#include "GarraOpen.h"

GarraOpen garra;

//int red = A0, blue = A1, green = A2;

void setup()
{
  /*
    pinMode (red, OUTPUT);
    pinMode (blue, OUTPUT);
    pinMode (green, OUTPUT);
    digitalWrite (red, LOW);
    digitalWrite (blue, HIGH);
    digitalWrite (green, LOW);
  */
  //        pin_stp, pin_dir, pin_rst, pin_slp, pin_ena, pin_m0, pin_m1, pin_m2, fim_curso, inicio_curso
  garra.iniciaY (2, 3, 4, 13, 5, 13, 13, 13, 13, A2);
  garra.iniciaX (8, 9, 7, 13, 6, 13, 13, 13, A1, A0);
  garra.zeraGarra ();
  garra.iniciaServos(A3, A4, A5);
  Serial.begin (9600);

}

bool estado = HIGH;
int thisByte = 33;
void loop()
{

  char novaPos;
  int pos;

  //digitalWrite (blue, estado);
  //Le o valor da Chave Esquerda (On/Off)
  if (Serial.available ()) {
    novaPos = Serial.read();
    switch (novaPos) {
      case 'x':
        Serial.print ("x: \t");
        while (!Serial.available ()) {}
        pos = Serial.parseInt();
        Serial.println (pos);
        garra.moveX (pos);
        break;
      case 'y':
        Serial.print ("y: \t");
        while (!Serial.available ()) {}
        pos = Serial.parseInt();
        Serial.println (pos);
        garra.moveY (pos);
        break;
      case 'c':
        Serial.print ("cot: \t");
        while (!Serial.available ()) {}
        pos = Serial.parseInt();
        Serial.println (pos);
        garra.moveCotovelo (pos);
        break;
      case 'p':
        Serial.print ("pul: \t");
        while (!Serial.available ()) {}
        pos = Serial.parseInt();
        Serial.println (pos);
        garra.movePulso (pos);
        break;
      case 'a':
        Serial.print ("atu: \t");
        while (!Serial.available ()) {}
        pos = Serial.parseInt();
        Serial.println (pos);
        garra.moveAtuador (pos);
        break;
      case 'o':
        Serial.println ("3500x");
        garra.passoX ();
        break;
      case 'l':
        Serial.println ("3500y");
        garra.passoY ();
        break;
      case 't':
        Serial.println ("vaiLoko");
        garra.segueTrajetoria (0);
        break;
      case 'g':
        Serial.println ("vaiLokoVOlta");
        garra.segueTrajetoria (1);
        break;
      default:
        break;
    }
  }
  //*/
}







