#include <math.h>
#include "driver.h"

// Defines -------------------------------------------------------------------------------------------------

// Encoders esquerda
#define encoder0PinA 2

// Encoders direita
#define encoder1PinA 3

// Calculo de velocidade
#define pi 3.14159265359f

// Controle de velocidade
#define limiar_erro_velocidade 0.07f
#define Kp_esquerda 6.5f
#define Ki_esquerda 0.05f
#define Kd_esquerda 0.1f
#define Kp_direita 6.5f
#define Ki_direita 0.05f
#define Kd_direita 0.1f

#define VELOCIDADE_MAXIMA 5.5f
#define VELOCIDADE_LIMITE 8.0f

// Variáveis -----------------------------------------------------------------------------------------------


// Encoders
volatile long encoder0Pos = 0; //esquerda
volatile long encoder1Pos = 0; //direita
double voltas_esquerda = 0, voltas_esquerda_anterior = 0;
double voltas_direita = 0, voltas_direita_anterior = 0;

// Cálculo de velocidade
float velocidade_esquerda = 0;
float velocidade_direita = 0;
int velocidade_esquerda_aux = 0;
int velocidade_direita_aux = 0;
double tempo;
double tempo_aux;

// Controle de velocidade
float velocidade_ReferenciaDireita_anterior = 0;
float velocidade_ReferenciaEsquerda_anterior = 0;

//parametros do controlador, talvez mexendo aqui melhore a diferença entre as rodas ass: Luan
//Mexer neles muda tipo a resposta do motor, o estado transitório e o Sobressinal da resposta Ass: Letícia
//Se vc consegui deixar as respostas exatamente iguais, talvez melhore Ass: Letícia
float theta1_esquerda = 0.40;
float theta2_esquerda = 0.15;//0.25;
float yTv_esquerda = 0.08;

float theta1_direita = 0.40;
float theta2_direita = 0.15;
float yTv_direita = 0.07;

float velocidade_esquerda_modelo = 0;
float velocidade_direita_modelo = 0;
float tensaomotor_esquerda = 0;
float tensaomotor_direita = 0;
float tensao_bateria = 12;

//desacelerar
float potencia_aux_esquerda = 0;
float potencia_aux_direita = 0;
float aceleracao_Esquerda =0;
float aceleracao_Direita = 0;


//Funcoes -----------------------------------------------------------------------------------------------------------------------------------

void doEncoderA();
void doEncoder1A();

//Funcoes de setup dos módulos --------------------------------------------------------------------------------------------------------------

void startEncoder () {

  pinMode(encoder0PinA, INPUT);
  pinMode(encoder1PinA, INPUT);
  // Encoder Esquerda
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  // Encoder Direita
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
}

//Funções Encoder -------------------------------------------------------------------------------------------------------------

void doEncoderA() {

  noInterrupts();
  encoder0Pos = encoder0Pos + 1;
  interrupts();
}

void doEncoder1A() {
  noInterrupts();
  encoder1Pos = encoder1Pos + 1;
  interrupts();
}

// Funcoes Controle de Velocidade -----------------------------------------------------------------------------------------
void desacelerar (){
  aceleracao_Esquerda = (-potencia_aux_esquerda)/0.2; 
  aceleracao_Direita = (-potencia_aux_direita)/0.2;
  
  pot_esquerda = potencia_aux_esquerda + (aceleracao_Esquerda*millis()/1000);
  pot_direita = potencia_aux_direita + (aceleracao_Direita*millis()/1000); 
  
  if ((potencia_aux_esquerda > 0 && pot_esquerda < 0) || (potencia_aux_esquerda < 0 && pot_esquerda > 0) || (abs(pot_esquerda) < POT_MIN_ESQUERDA)){
    parar();
    pot_esquerda = 0;
    pot_direita = 0;  
  }
  else{
    setVelocidade();  
  }
 
}
void controleAdaptativoVelocidade() {

  if (velocidade_ReferenciaEsquerda == 0 && velocidade_ReferenciaDireita == 0) {

    velocidade_ReferenciaEsquerda_anterior = velocidade_ReferenciaEsquerda;
    velocidade_ReferenciaDireita_anterior = velocidade_ReferenciaDireita;
    
    theta1_esquerda = 0.40;
    theta2_esquerda = 0.15;
    yTv_esquerda = 0.08;

    theta1_direita = 0.40;
    theta2_direita = 0.15;
    yTv_direita = 0.07;

    //parar();
    desacelerar();
    return;
  }

  theta1_esquerda = theta1_esquerda - (yTv_esquerda * velocidade_ReferenciaEsquerda * (velocidade_esquerda - velocidade_esquerda_modelo));
  theta2_esquerda = theta2_esquerda + (yTv_esquerda * velocidade_esquerda * (velocidade_esquerda - velocidade_esquerda_modelo));

  theta1_direita = theta1_direita - (yTv_direita * velocidade_ReferenciaDireita * (velocidade_direita - velocidade_direita_modelo));
  theta2_direita = theta2_direita + (yTv_direita * velocidade_direita * (velocidade_direita - velocidade_direita_modelo));

  tensaomotor_esquerda = (theta1_esquerda * velocidade_ReferenciaEsquerda) - (theta2_esquerda * velocidade_esquerda);

  tensaomotor_direita = (theta1_direita * velocidade_ReferenciaDireita) - (theta2_direita * velocidade_direita);

  velocidade_esquerda_modelo = (0.00248 * velocidade_esquerda_modelo) + (0.99752 * velocidade_ReferenciaEsquerda_anterior);

  velocidade_direita_modelo = (0.00248 * velocidade_direita_modelo) + (0.99752 * velocidade_ReferenciaDireita_anterior);

  velocidade_ReferenciaEsquerda_anterior = velocidade_ReferenciaEsquerda;
  
  velocidade_ReferenciaDireita_anterior = velocidade_ReferenciaDireita;

  pot_esquerda = 100 * tensaomotor_esquerda / (tensao_bateria);
  
  
  //TODO compensar a deadzone
  pot_direita = 100 * tensaomotor_direita / (tensao_bateria);
  
  potencia_aux_esquerda = pot_esquerda;
  potencia_aux_direita = pot_direita;

  setVelocidade();
  return;
}
// Setup e Loop principais----------------------------------------------------------------------------------------

void setup() {
  tempo = millis();
  Serial.begin(115200);
  startDriver();
  startEncoder();
}

void loop() {
   // Calcula as velocidades das rodas a cada 20 ms ---------------------------------
   if (millis() - tempo > 20) 
   {      
      tempo_aux = (millis() - tempo);
      tempo = millis();
    
      voltas_esquerda = encoder0Pos / (4192.0); // 32*131 - mudamos para 32 pq pega a descida tb
      voltas_direita = encoder1Pos / (4192.0); // tem que ter o .0 para o arduino não interpretar como inteiro e ferrar com a precisão
    
      velocidade_esquerda = 1000 * (voltas_esquerda - voltas_esquerda_anterior) / (tempo_aux);
      velocidade_direita = 1000 * (voltas_direita - voltas_direita_anterior) / (tempo_aux);
    
      voltas_esquerda_anterior = voltas_esquerda;
      voltas_direita_anterior = voltas_direita;
  }
  // Chama a funcao que calcula a tensao de saida para os motores -----------------------
  controleAdaptativoVelocidade();
  // ------------------------------------------------------------------------------------
  
  // Recebe velocidade  de entrada da serial do arduino ---------------------------------------

  if (Serial.available() > 0) 
  {
    velocidade_ReferenciaEsquerda_anterior = velocidade_ReferenciaEsquerda;
    velocidade_ReferenciaDireita_anterior = velocidade_ReferenciaDireita;
    velocidade_ReferenciaDireita = Serial.parseFloat();
    velocidade_ReferenciaEsquerda = velocidade_ReferenciaDireita * 1.115;//alteração para o robô poder andar mais reto
    Serial.print("Velocidade recebida: ");      Serial.println(velocidade_ReferenciaDireita);
  }
  // --------------------------------------------------------------------------------------
  
  // Recebe velocidade do outro arduino ---------------------------------------------------------------
}
