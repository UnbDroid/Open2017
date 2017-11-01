#include "GarraOpen.h"

GarraOpen garra;

int red = A0, blue = A1, green = A2;

void setup()
{
  pinMode (red, OUTPUT);
  pinMode (blue, OUTPUT);
  pinMode (green, OUTPUT);
  digitalWrite (red, LOW);
  digitalWrite (blue, HIGH);
  digitalWrite (green, LOW);
  garra.iniciaX (2, 3, 8, 9, 7, 4, 5, 6, 10, 12);
  garra.iniciaServos(0,0,11);
  garra.moveX(5);
  garra.zeraGarra ();
  Serial.begin (9600);

}

bool estado = HIGH;
int thisByte = 33;
void loop()
{

  char novaPos;

  
  digitalWrite (blue, estado);
  //Le o valor da Chave Esquerda (On/Off)
  if (Serial.available ()) {
    novaPos = Serial.read();
    if (novaPos == 'm') {
      while (!Serial.available ()) {}
      novaPos = Serial.read();
      garra.seVira (novaPos, 0, 0, 0, 0);
      Serial.print ('m');
    }
    else if (novaPos == 'l') {
      Serial.print ('l');
      //estado = !estado;
    }
    else if (novaPos == 'f'){
      garra.seVira (0, 0, 0, 0, 1);
      estado = !estado;
    }
  }
  //*/
}







