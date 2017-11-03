#include "opencv2/opencv.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
using namespace cv;

bool copoFora (int posGarra, VideoCapture cap);
bool copoDentro (int posGarra);
bool identificaCopo (Mat imagem);
bool andaRobo();
bool corrigePosGarra (int posicao_garra);


int main (){
  VideoCapture cap(0);
  if(!cap.isOpened())  // check if we succeeded
      return -1;
  int a = copoFora (250, cap);

  //b = copoDentro (296)
  cap.release();
  destroyAllWindows();
}

bool corrigePosGarra (int posicao_garra){
  return 0;
}
bool andaRobo(){
  return 0;
}

bool identificaCopo (Mat imagem){
  Scalar area = sum (imagem);
  namedWindow("ide", WND_PROP_OPENGL);
  imshow("ide", imagem);
  printf ("Area = %lf\n", area[0]);
  if (area[0] > 1.0)
      return 1;
  return 0;
}
bool copoFora (int posGarra, VideoCapture cap){
  int pos_real, posicao_garra;
  pos_real = 0;
  posicao_garra = posGarra;

  while (true){
    Mat frame, imgCortada, binDirH;

    //for i in range (0, 19):
      //retr = cap.grab()
      //retr, frame = cap.retrieve(retr)

    cap >> frame; // get a new frame from camera
    imshow("frame2", frame);
    cvtColor(frame, frame, COLOR_BGR2HSV);


    Rect corteDir;
    corteDir.x = posicao_garra+25; //210
    corteDir.width  = 80; //290
    corteDir.y = 420;
    corteDir.height = 475 - corteDir.y;
    if ((corteDir.x+corteDir.width)>frame.cols){
      printf ("Acabou e nao achou\n");
      return 0;
    }
    imgCortada = frame (corteDir);

    cvtColor(frame , frame, COLOR_HSV2BGR);

    rectangle(frame,Point2f(corteDir.x,corteDir.y),Point2f (corteDir.x+corteDir.width,corteDir.y+corteDir.height),(0,255,0),3,8,0);
    namedWindow("frame", WND_PROP_OPENGL);
    imshow("frame", frame);
    waitKey();

    vector<Mat> channels2;
    split(imgCortada,channels2);//Separa canais da imagem

    threshold(channels2[0], binDirH, 60,1, THRESH_BINARY_INV);

    int copoNaDireita = identificaCopo (binDirH);

    //char = cv2.waitKey(delay);
    //cv2.namedWindow('imgCopoDireita', cv2.WND_PROP_OPENGL);
    //cv2.imshow('imgCopoDireita', imgCopoDireita);

    //cv2.namedWindow('Hsvframe', cv2.WINDOW_OPENGL);
    //cv2.imshow('Hsvframe',dirH);

    namedWindow("thresh1", WINDOW_OPENGL);
    Mat bin2 = binDirH*255;
    imshow("thresh1",bin2);

    if (copoNaDireita == 0)//# and copoNaEsquerda == 0:
    {
      if (posicao_garra == posGarra)
      {
        posicao_garra += 35;
        pos_real += 30;
        corrigePosGarra (pos_real);
      }
      else{
        pos_real += 5;
        corrigePosGarra (pos_real);
        return 1;
      }
    }
    else if (copoNaDireita == 1){
      posicao_garra += 9;
      pos_real += 11;
      corrigePosGarra (pos_real);
    }
  }
}
bool copoDentro (int posGarra, VideoCapture cap){
    int posicao_garra = posGarra; // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return 0;

    Mat edges;
    for(;;)
    {
        Mat frame, imgCortada, binH;
        cap >> frame; // get a new frame from camera
        imshow("frame", frame);
        cvtColor(frame, frame, COLOR_BGR2HSV);


        Rect corte;

        corte.x = posicao_garra-30; //210
        corte.width  = 60; //290
        corte.y = 415;
        corte.height = 450 - corte.y;
        if ((corte.x+corte.width)>frame.cols){
          printf ("Acabou\n");
          return 0;
        }
        imgCortada = frame (corte);
        namedWindow("imgCortada", WND_PROP_OPENGL);
        imshow("imgCortada", imgCortada);

        vector<Mat> channels2;
        split(imgCortada,channels2);//Separa canais da imagem

        threshold(channels2[0], binH, 60,1, THRESH_BINARY_INV);

        if (identificaCopo (binH) == 0){
            return 1;
        }else{
      		waitKey();
        }
      }
          //andaRobo ()
}
