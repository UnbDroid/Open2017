#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <vector>
#include "arduino_msgs/StampedInt32.h"
#include "opencv2/opencv.hpp"


#define QUANTIDADE_MOTORES_GARRA 1

#define MOTOR_PASSO_X 0 //MOVE MOTOR NO EIXO X PARA SE ALINHAR
#define ANDA_ROBO 1  //MANDA ROBO IR PARA FRENTE PARA PEGAR COPO
#define ACABOU_GARRA 2 //INFORMA QUE ACABOU MOVIMENTAÇÃO

#define ANGULO_TANQUE 3 //ANGULO DO ROBO AO TANQUE
#define DISTANCIA_TANQUE 4 //DISTANCIA DO ROBO AO TANQUE

#define DISTANCIA_AVANÇO_ROBO 20

#define TANQUE 10
#define GARRA 20

#define NUM_IDEN_VISAO 600


using namespace cv;
using namespace std;

ros::Publisher pubEstrategia_int32;

ros::Subscriber subEstrategia_int32;

bool semComando, ocupado;
int comandoRecebido, angTanque, distTanque, flagRespostaMega;

bool copoFora (int posGarra, VideoCapture cap);
bool copoDentro (int posGarra);
bool identificaCopo (Mat imagem);
bool andaRobo();
bool corrigePosGarra (int posicao_garra);
void setGlobal ();
void initROS(ros::NodeHandle nh);
void estrategiaVCTCB(const arduino_msgs::StampedInt32& VCestrategia_msg);
void posicaoTanque (int line1size,  int px1, float center);
void achaTanque(VideoCapture cap);

/*
estrutura de loop recomendada

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    algoritmo();
    ros::spinOnce();
    loop_rate.sleep();
  }

*/
//while(ocupado)

int main (int argc, char **argv){
  ros::init(argc, argv, "visao_copo_tanque_node");
  ros::NodeHandle nh;
  initROS(nh);
  
  //atualizacao de callbacks
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    while (semComando) //Espera comando
    {
      ros::spinOnce();
      loop_rate.sleep();  
    }

    VideoCapture cap(0); //Abre camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    if (comandoRecebido == TANQUE){ //se comando recebido for para identificar o tanque
      if (tanque(cap)){ //chama função do tanque e se ela achou tanque
        SendIntEstrategia(NUM_IDEN_VISAO+ANGULO_TANQUE, angTanque); //Retorna variaveis globais
        SendIntEstrategia(NUM_IDEN_VISAO+DISTANCIA_TANQUE, distTanque);//Retorna variaveis globais
      }
      else {//caso nao achou o tanque envia valor bugadaço
        SendIntEstrategia(NUM_IDEN_VISAO+ANGULO_TANQUE, 180); //Retorna valor de erro
        SendIntEstrategia(NUM_IDEN_VISAO+DISTANCIA_TANQUE, -1);//Retorna valor de erro  
      }
    }
    else {
      //Se alinha com copo e recebe 1 se tudo der certo, passa posição do pixel que a garra esta na camera 
      //(CASO DE ERRADO PEDIR AO BRUNO PARA ENCONTRAR POSIÇÃO DO INICIO A ESQUERDA DO COPO)
      int a = copoFora (350, cap); 
      if (a) //Se o copo tiver sido alinhado
        a = copoDentro (); //Move o robo para frente para pegar o copo
      SendIntEstrategia(NUM_IDEN_VISAO+ACABOU_GARRA, a); //Envia que terminou movimentação e se deu tudo certo
    }

    cap.release(); //Fecha camera
    destroyAllWindows();
  }
}

void estrategiaVCTCB(const arduino_msgs::StampedInt32& VCestrategia_msg)
{
  switch (VCestrategia_msg.id-NUM_IDEN_VISAO){
    case 0: //Recebe informação do arduino;
      flagRespostaMega = VCestrategia_msg.data;
      ocupado = false;
      break;
    case 1: //Recebe informação do arduino;
      flagRespostaMega = VCestrategia_msg.data;
      ocupado = false;
      break;
    case 10: //CONSIDEREI SOMENTE O ID SUFICIENTE PARA A VISAO SABER O QUE FAZER
      semComando = false;
      comandoRecebido = TANQUE;
      break;
    case 20: //CONSIDEREI SOMENTE O ID SUFICIENTE PARA A VISAO SABER O QUE FAZER
      semComando = false;
      comandoRecebido = GARRA;
      break;
    default:
      break;
  }
}

void initROS(ros::NodeHandle nh)
{
    pubEstrategia_int32 = nh.advertise<arduino_msgs::StampedInt32>("VCestrategia", 1000);
    subEstrategia_int32 = nh.subscribe("estrategiaVCT", 1000, estrategiaVCTCB);
}

void setGlobal (){
  comandoRecebido = 0;
  semComando = false;
  ocupado = false;
}

void SendIntEstrategia(int id, long int data)
{    
  arduino_msgs::StampedInt32 int32_msg;
  int32_msg.id = id;
  int32_msg.data = data;
  pubEstrategia_int32.publish(int32_msg);
}

//ESSA É A FUNÇÃO PARA MOVIMENTAR EIXO DA GARRA
bool corrigePosGarra (int posicao_garra){
  SendIntEstrategia(NUM_IDEN_VISAO + MOTOR_PASSO_X, posicao_garra); //ENVIA A POSIÇÃO DESEJADA
  ocupado = true;
  while (ocupado) //Espera confirmação do arduino do fim do movimento
  {
    ros::spinOnce();
    loop_rate.sleep();  
  }
  return flagRespostaMega;
}
bool andaRobo(){
  //ENVIA PARA ROBO ANDAR UMA DISTANCIA FIXA E PEQUENA
  SendIntEstrategia(NUM_IDEN_VISAO + ANDA_ROBO, DISTANCIA_AVANÇO_ROBO); 
  ocupado = true;
  while (ocupado) //Espera confirmação do arduino do fim do movimento
  {
    ros::spinOnce();
    loop_rate.sleep();  
  }
  return flagRespostaMega;
}


//SE PEGOU UMA REGIÃO DE AZUL RETORNA QUE O LED PEGOU O COPO
bool identificaCopo (Mat imagem){
  Scalar area = sum (imagem);
  //printf ("Area = %lf\n", area[0]);
  if (area[0] > 1.0)
      return 1;
  return 0;
}

//FUNÇÃO PARA SE ALINHAR COM O COPO
bool copoFora (int posGarra, VideoCapture cap){
  int pos_real, posicao_garra;
  pos_real = 0;
  posicao_garra = posGarra;

  while (true){
    Mat frame, imgCortada, binDirH;

    cap >> frame; // get a new frame from camera
    cvtColor(frame, frame, COLOR_BGR2HSV);


    Rect corteDir;
    corteDir.x = posicao_garra+25; //POSIÇÃO X DO INICO DO COPO //210
    corteDir.width  = 80; // LARGURA DO COPO//290
    corteDir.y = 420; //INICIO DA ALTURA DO COPO (Y CRESCENTE DE CIMA PARA BAIXO)
    corteDir.height = 475 - corteDir.y; //ALTURA DO COPO
    
    if ((corteDir.x+corteDir.width)>frame.cols){//Se a camera varreu ate o final da imagem, nao achou copo
      //printf ("Acabou e nao achou\n");
      return 0;
    }
    imgCortada = frame (corteDir);
    //imshow("imgCortada", imgCortada);

    cvtColor(frame , frame, COLOR_HSV2BGR);


    vector<Mat> channels2;
    split(imgCortada,channels2);//Separa canais da imagem

    threshold(channels2[2], binDirH, 230/*60*/,1, THRESH_BINARY);
    rectangle(frame,Point2f(corteDir.x,corteDir.y),Point2f (corteDir.x+corteDir.width,corteDir.y+corteDir.height),(0,255,0),3,8,0);

    int copoNaDireita = identificaCopo (binDirH);

    if (copoNaDireita == 0)//# and copoNaEsquerda == 0:
    {
      if (posicao_garra == posGarra) //SE COPO NAO FOI ENCONTRADO LOGO NO INICIO, SE MOVA PARA A ESQUERDA E REPROCURE
      {
        posicao_garra += 35; //ANDA 35 PIXEIS A DIREITA NA IMAGEM
        pos_real += 30; //MOVE 30 MM NO MUNDO REAL
        bool resposta = corrigePosGarra (pos_real);
        if (resposta == 0){
          return 0;
        }
      }
      else{ //SE NAO FOI ENCOTRADO COPO EM UMA POSIÇÃO DIFERENTE DA INCIAL, ASSUME QUE O ACHOU
        pos_real += 5; //DA UMA AJUSTADA
        bool resposta = corrigePosGarra (pos_real);
        if (resposta == 0){
          return 0;
        }
        return 1; //ALINHOU COM O COPO
      }
    }
    else if (copoNaDireita == 1){ //SE LED AINDA BATEU NO COPO
      posicao_garra += 9; //MOVE 9 PIXEIS A DIREITA
      pos_real += 11; //MOVE 11 MM NO MUNDO
      bool resposta = corrigePosGarra (pos_real);
      if (resposta == 0){
        return 0;
      }
    }
  }
}

//FUNÇÃO PARA AJUSTAR COPO DENTRO DA GARRA
/*
FUNÇÃO NAO TESTADA
*/

bool copoDentro (int posGarra, VideoCapture cap){
    int posicao_garra = posGarra; // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return 0;

    Mat edges;
    for(;;)
    {
        Mat frame, imgCortada, binH;
        cap >> frame; // get a new frame from camera
        //imshow("frame", frame);
        cvtColor(frame, frame, COLOR_BGR2HSV);


        Rect corte;

        corte.x = posicao_garra-30; //210
        corte.width  = 60; //290
        corte.y = 415;
        corte.height = 450 - corte.y;
        if ((corte.x+corte.width)>frame.cols){
          //printf ("Acabou\n");
          return 0;
        }
        imgCortada = frame (corte);
        //namedWindow("imgCortada", WND_PROP_OPENGL);
        //imshow("imgCortada", imgCortada);

        vector<Mat> channels2;
        split(imgCortada,channels2);//Separa canais da imagem

        threshold(channels2[0], binH, 60,1, THRESH_BINARY_INV);

        if (identificaCopo (binH) == 0){
         return 1;
        }else{
      	 andaRobo ();
        }
    }
}




bool achaTanque(VideoCapture cap)
{
    if(!cap.isOpened()){  // check if we succeeded
        printf ("Erro na camera\n");
        return 0;
      }

    //for(;;)
    //{
        Mat frame, equalizado, binario;
        cap >> frame; // get a new frame from camera
        //imshow("frame", frame);

        //Mat matretangulos = frame.clone();
        //Mat contornos = frame.clone();
        

        //Equaliza cie ------------------------------
        Mat cie, cie2;
        cvtColor(frame,cie,CV_BGR2Lab);//Converte para Cie L*a*b
        vector<Mat> channels2;
        split(cie,channels2);//Separa canais da imagem
        equalizeHist(channels2[0],channels2[0]);//Equaliza canal L
        merge(channels2,equalizado);//Reune canais novamente

        //Converte para BGR
        cvtColor(equalizado,frame,CV_Lab2BGR);
        //imshow( "equalizado", equalizado);
        inRange(equalizado,Scalar(0,145,140), Scalar(250,190,175),binario);


        //imshow( "binario", binario);
        vector<vector<Point> > contours;  //Para encontrar contornos
        vector<Vec4i> hierarchy;          //Para encontrar contornos

        findContours( binario, contours, hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

        //Variaveis contorno
        vector<vector<Point> > contours_poly( contours.size());//Para encontrar contornos
        vector<Rect> boundRect( contours.size() );      //Para encontrar retangulo
        Rect maiorRetangulo;

        RNG rng (12345);

        for( int i = 0; i < contours.size(); i++ ){                       //Para cada contorno
          approxPolyDP( Mat(contours[i]), contours_poly[i],5, true ); //Aproxima contorno por um poligono
          boundRect[i] = boundingRect( Mat(contours_poly[i]) );              //Menor retangulo rotacionado capaz de conter o contorno
         }

         int maxArea = 0, indTanque;
         Scalar color = Scalar (rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
         for( int i = 0; i< contours.size(); i++ ){
           if ((boundRect[i].width+boundRect[i].height)>(frame.rows+frame.cols)/50&&(boundRect[i].width+boundRect[i].height)<(frame.rows+frame.cols)/1.2){
             double acontours = contourArea(contours_poly[i], false);
             if ((boundRect[i].area()/acontours)<2){
               if (acontours>maxArea){
                 maxArea = acontours;
                 indTanque = i;
                 maiorRetangulo = boundRect[i];
                }
             }
           }
         }

        float razao = maiorRetangulo.width;
        razao /= maiorRetangulo.height;
        if (razao<1.9){
           posicaoTanque (2*maiorRetangulo.height, maiorRetangulo.x, frame.cols/2);
         }
         else {
           posicaoTanque (maiorRetangulo.width, maiorRetangulo.x, frame.cols/2);
         }
         //rectangle( matretangulos, maiorRetangulo.tl(), maiorRetangulo.br(), Scalar (255, 0, 0), 2, 8, 0 );
         //imshow( "Tanque", matretangulos);
         //printf ("Razao %f\n", (50*maiorRetangulo.width)/(maiorRetangulo.height));
         //printf ("Angulo: %lf\tDistancia: %f\n", angTanque, distTanque);
         //if(waitKey(30) >= 0) break;
    //}
    // the camera will be deinitialized automatically in VideoCapture destructor
   return 1;
}



void posicaoTanque (int line1size,  int px1, float center){
  //printf ("line %d\n", line1size);
  float z1 = 412.251-1.6477*float (line1size)+0.0018803*float (line1size*line1size);//-56/line1size;
  float x1 = (z1*(px1-center)/FOCAL_DIST)+X_ERROR_CONST;
  angTanque = int (atan (x1/z1)*360/3.141592);
  distTanque = int (pow ((pow (x1, 2) + pow (z1, 2)), 0.5));
}
