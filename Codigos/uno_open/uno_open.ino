#include "roscom.h"
#include "contvel.h"
#include "gyro.h"

void messageInt32Cb( const arduino_msgs::StampedInt32& r_int32_msg)
{   
  if(r_int32_msg.id == GIRA){
     STATE = 0;
     graus = r_int32_msg.data;      
  }  
}

void messageFloat32Cb( const arduino_msgs::StampedFloat32& r_float32_msg)
{
  //velocidade_ReferenciaEsquerda = velocidade_ReferenciaDireita * 1.115;//alteração para o robô poder andar mais reto
  
  if(r_float32_msg.id == VEL_REF_DIR){
     if(r_float32_msg.data>0){
        if(dir==0) settavaloresIniciaisParametros();
        dir = 1;
     }
     if(r_float32_msg.data<0){
        if(dir==1) settavaloresIniciaisParametros();
        dir = 0;
     }    
     velocidade_ReferenciaDireita_anterior = velocidade_ReferenciaDireita;
     velocidade_ReferenciaDireita = abs(r_float32_msg.data);
     //Serial.print("Velocidade recebida Dir: ");      Serial.println(velocidade_ReferenciaDireita);
  }else if(r_float32_msg.id == VEL_REF_ESQ){
    
     if(r_float32_msg.data>0){
        if(dir==0) settavaloresIniciaisParametros();
        dir = 1;
     }
     if(r_float32_msg.data<0){
        if(dir==1) settavaloresIniciaisParametros();
        dir = 0;
     }      
     velocidade_ReferenciaEsquerda_anterior = velocidade_ReferenciaEsquerda;
     velocidade_ReferenciaEsquerda = abs(r_float32_msg.data);//* 1.115;  para poder corrigir para andar reto 
     //Serial.print("Velocidade recebida Esq: ");      Serial.println(velocidade_ReferenciaEsquerda);  
  }  
}

void setup()
{
  StartVelCont();
  SetupGyroscope(2000);
  initializeROS();
  STATE = 1;
  dir = 1;
  pinMode(LED_BUILTIN, OUTPUT);
  //Serial.begin(115200); 
}

void loop()
{
  if(STATE)
    UpdateVel();
  else
    Turn(graus);
  nh.spinOnce();
}
