#include "roscom.h"
#include "contvel.h"
#include "giro.h"

void messageInt32Cb( const arduino_msgs::StampedInt32& r_int32_msg)
{   
}

void messageFloat32Cb( const arduino_msgs::StampedFloat32& r_float32_msg)
{
  //velocidade_ReferenciaEsquerda = velocidade_ReferenciaDireita * 1.115;//alteração para o robô poder andar mais reto
  sendInt32(666, dir);
  if(r_float32_msg.id == VEL_REF_DIR){
     velocidade_ReferenciaDireita_anterior = velocidade_ReferenciaDireita;
     velocidade_ReferenciaDireita = abs(r_float32_msg.data);
     if(r_float32_msg.data>0){
        if(dir==0)  settavaloresIniciaisParametros();
        dir = 1;
        gDir = 1;
     }
     if(r_float32_msg.data<0){
        if(dir==1) settavaloresIniciaisParametros();
        dir = 0;
        gDir = 0;
     }    
  }else if(r_float32_msg.id == VEL_REF_ESQ){
     velocidade_ReferenciaEsquerda_anterior = velocidade_ReferenciaEsquerda;
     velocidade_ReferenciaEsquerda = abs(r_float32_msg.data);//* 1.3;  //para poder corrigir para andar reto 
     if(r_float32_msg.data>0){
        if(dir==0) settavaloresIniciaisParametros();
        dir = 1;
        gDir = 1;
     }
     if(r_float32_msg.data<0){
        if(dir==1) settavaloresIniciaisParametros();
        dir = 0;
        gDir = 0;
     }      
  }else if(r_float32_msg.id == TRAVAR){
      travar();
  }else if(r_float32_msg.id == GIRA){
     STATE = 0;
     graus = r_float32_msg.data;      
  }   
}

void setup()
{
  StartVelCont();
  inicializaGiro();
  initializeROS();
  STATE = 1;
  dir = 1;
  gDir = 1;
  sendInt32(6, dir);
}

void loop()
{
  if(STATE){
    UpdateVel();
  }
  else{
    Turn();
  }
  nh.spinOnce();
}
