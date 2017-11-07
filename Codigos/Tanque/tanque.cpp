#include "opencv2/opencv.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
using namespace cv;

//Macros:
#define FOCAL_DIST 602.1197
#define X_ERROR_MI 0.129152		//The x axis position of the analised line have an error that grows linearly (approximately) with the distance in the z axis
#define X_ERROR_CONST 0.138	//Mathematically: error = z1*X_ERROR_MI + X_ERROR_CONSTANT

double angTanque;
float distTanque;

#define NUM_IDEN_VISION 500
void posicaoTanque (int line1size,  int px1, float center);

int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened()){  // check if we succeeded
        printf ("Erro na camera\n");
        return -1;
      }

    for(;;)
    {
        Mat frame, equalizado, binario;
        cap >> frame; // get a new frame from camera
        //imshow("frame", frame);

        Mat matretangulos = frame.clone();
        Mat contornos = frame.clone();
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
         printf ("Angulo: %lf\tDistancia: %f\n", angTanque, distTanque);
         if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}



void posicaoTanque (int line1size,  int px1, float center){
  //printf ("line %d\n", line1size);
  float z1 = 412.251-1.6477*float (line1size)+0.0018803*float (line1size*line1size);//-56/line1size;
  float x1 = (z1*(px1-center)/FOCAL_DIST)+X_ERROR_CONST;
  angTanque = atan (x1/z1)*360/3.141592;
  distTanque = pow ((pow (x1, 2) + pow (z1, 2)), 0.5);
}
