#include "roscom.h"
#include "US.h"
#include "tasks.h"
#include "toque.h"
#include "GarraOpen.h"


//"ESTADOS" que a estrategia pode mandar para o mega
#define GARRA_DENTRO 1  //MOVE A GARRA PARA ORDENHA
#define GARRA_FORA 2 //TIRA GARRA DA ORDENHA
#define SENSORES 3 
#define DESPEJA 4 //MOVE GARRA PARA DESPEJAR 
#define PREPARA_COPO 5 //MOVE GARRA PARA POSICAO MAIS BAIXA COM ATUADOR ABERTO PARA ENTRADA OU SAIDA DE COPO
#define MAMADOR 6 //DEVE FECHAR O MAMADOR E MOVER O MOTOR DC

int STATE;
GarraOpen Garra;

void messageInt64Cb( const arduino_msgs::StampedInt64& r_int64_msg)
{   
  if(Garra.ehGarra(r_int64_msg.id))
  {
      int flag_resultado = Garra.trataGarra(r_int64_msg.id, r_int64_msg.data);
      sendInt64(r_int64_msg.id,flag_resultado);
  }
}

void messageFloat64Cb( const arduino_msgs::StampedFloat64& r_float64_msg)
{
}



void taskUSCallback1()
{
  readUS(FRENTE_R);
  readUS(TRAS_L);
  sendInt64( FRENTE_R + NUM_IDEN_US , USReadings[FRENTE_R]);
  sendInt64(TRAS_L + NUM_IDEN_US , USReadings[TRAS_L]);
}

void taskUSCallback2()
{
  readUS(FRENTE_L);
  readUS(TRAS_R);
  sendInt64(FRENTE_L + NUM_IDEN_US , USReadings[FRENTE_L]);
  sendInt64(TRAS_R + NUM_IDEN_US , USReadings[TRAS_R]);

}

void taskUSCallback3()
{
  readUS(CIMA_R);
  readUS(LADO_L);
  sendInt64(CIMA_R + NUM_IDEN_US , USReadings[CIMA_R]);
  sendInt64(LADO_L + NUM_IDEN_US , USReadings[LADO_L]);
}

void taskUSCallback4()
{
  readUS(CIMA_L);
  readUS(LADO_R);
  sendInt64(CIMA_L + NUM_IDEN_US , USReadings[CIMA_L]);
  sendInt64(LADO_R + NUM_IDEN_US , USReadings[LADO_R]);
}

void taskTOQUECallback()
{
   lerSensoresToque();
   int i = 0;
   while(i<QUANTIDADE_SENSOR_TOQUE)
   {
        if(toque[i]!=oldToque[i]){
             sendInt64(i + NUM_IDEN_TOQUE, toque[i]);
             oldToque[i] = toque[i];
        }
        i++;
   }
}

//8 sensores US
//9 sensores de toque

void setup()
{
  initializeROS();
  startSENSORTOQUE();
  //Garra.setupGarra(); //OBSERVAÇÃO!!!!!!!!!!!!!!!! PODE HAVER LOOP INFINITO AQUI SE A GARRA NAO TOCAR NOS SENSORES DE FIM DE CURSO
  start_TASKS();
  STATE = SENSORES;
}

void loop()
{
  switch(STATE)
  {
      case GARRA_DENTRO: //Move a garra para posição de ordenha
          //Garra.segueTrajetoria(0);
          break;
      case GARRA_FORA //Move a garra para fora da posição de ordenha, para despejar ou devolver
          //Garra.segueTrajetoria(1);
          break;
      case DESPEJA: //Move a garra para despejar
          //Garra.despeja();
          break;
      case PREPARA_COPO: //Move a garra para pegar ou devolver o copo, obs: Atuador fica aberto
          //Garra.pega();
          break;
      case MAMADOR: //Função para ordenhar a vaca
          break;
      case SENSORES:
          runner.execute();
          break;
      default:
          runner.execute();
          break;
  }
  nh.spinOnce();
}
