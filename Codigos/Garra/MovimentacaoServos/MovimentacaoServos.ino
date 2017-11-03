#include <Servo.h>

Servo cotovelo, pulso, atuador;

void setup()
{
  // Pino de dados do servo conectado ao pino 9 do Arduino
  cotovelo.attach(10);
  pulso.attach(11);
  atuador.attach(12);
  Serial.begin (9600);
}

void loop()
{
  int valor;
  char serv;
  //Le o valor da Chave Esquerda (On/Off)
  if (Serial.available ()) {
    serv = Serial.read();
    switch (serv) {
      case 'c':
        Serial.println ("Cotovelo");

        while (!Serial.available ()) {}
        valor = Serial.parseInt();
        Serial.write (valor);

        cotovelo.write(valor);
        break;
      case 'p':
        Serial.println ("Pulso");
        
        while (!Serial.available ()) {}
        valor = Serial.parseInt();
        Serial.write (valor);
        
        pulso.write(valor);
        break;
      case 'a':
        Serial.println ("Atuador");
        
        while (!Serial.available ()) {}
        valor = Serial.parseInt();
        Serial.write (valor);
        
        atuador.write(valor);
        break;
      default:
        Serial.println ("Erro");
        break;
    }
    delay(15);          //Delay para o servo atingir a posiçao
    Serial.write (valor);
    Serial.print ('\n');
  }
}
