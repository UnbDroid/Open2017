#include "roscom.h"
#include "contvel.h"
#include "giro.h"
#include "camera.h"

void messageInt32Cb( const arduino_msgs::StampedInt32& r_int32_msg)
{   
}

void messageFloat32Cb( const arduino_msgs::StampedFloat32& r_float32_msg)
{
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
  initializeROS();
  InitCamera();
  StartVelCont();
  inicializaGiro();  
  STATE = 1;
  dir = 1;
  gDir = 1;
}

void loop()
{
  if(STATE == 1){
    UpdateVel();
  }
  else if(STATE == 0){
    Turn();
  } else if(STATE == 2){
    ChangeCamState();
  }
  nh.spinOnce();
}
