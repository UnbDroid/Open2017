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
void posicaoTanque (float line1size,  float px1, float center);

int main(int, char**)
{
    VideoCapture cap(1); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    for(;;)
    {
        Mat frame, equalizado, binario;
        cap >> frame; // get a new frame from camera
        imshow("frame", frame);
        //printf ("1\n");
        Mat matretangulos = frame.clone();
        Mat contornos = frame.clone();
        //Equaliza cie ------------------------------
        Mat cie, cie2;
        cvtColor(frame,cie,CV_BGR2Lab);//Converte para Cie L*a*b
        vector<Mat> channels2;
        split(cie,channels2);//Separa canais da imagem
        equalizeHist(channels2[0],channels2[0]);//Equaliza canal L
        merge(channels2,equalizado);//Reune canais novamente

        //printf ("2\n");
        //Converte para BGR
        cvtColor(equalizado,frame,CV_Lab2BGR);
        //imshow( "equalizado", equalizado);
        inRange(equalizado,Scalar(0,145,140), Scalar(250,190,175),binario);


        imshow( "binario", binario);
        vector<vector<Point> > contours;  //Para encontrar contornos
        vector<Vec4i> hierarchy;          //Para encontrar contornos

        findContours( binario, contours, hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

        //printf ("4\n");
        //Variaveis contorno
        vector<vector<Point> > contours_poly( contours.size());//Para encontrar contornos
        vector<RotatedRect>    minRect( contours.size());      //Para encontrar retangulo rotacionado


        //printf ("5\n");
        RNG rng (12345);

        for( int i = 0; i < contours.size(); i++ ){                       //Para cada contorno
          approxPolyDP( Mat(contours[i]), contours_poly[i],5, true ); //Aproxima contorno por um poligono
          minRect[i] = minAreaRect( Mat(contours_poly[i]) );              //Menor retangulo rotacionado capaz de conter o contorno
         }
         //imshow( "binario2", contornos);
         int maxArea = 0, indTanque;
         Scalar color = Scalar (rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
         Point2f recTanque[4], rect_points[4];
         for( int i = 0; i< contours.size(); i++ ){

           minRect[i].points( rect_points );
           double line1, line2;
           line1 = norm (rect_points[0]-rect_points[1]);
           line2 = norm (rect_points[0]-rect_points[3]);
           if ((line1+line2)>(frame.rows+frame.cols)/50&&(line1+line2)<(frame.rows+frame.cols)/1.5){
             double acontours = contourArea(contours_poly[i], false);
             if ((line1*line2/acontours)<2){
               if (acontours>maxArea){
                 maxArea = acontours;
                 indTanque = i;
                 if (rect_points[1].y>rect_points[3].y&&rect_points[2].y>rect_points[0].y){

                   recTanque[0] = rect_points[1];
                   recTanque[1] = rect_points[2];
                   recTanque[2] = rect_points[0];
                   recTanque[3] = rect_points[3];

                 }
                 else if (rect_points[0].y<rect_points[2].y){

                   recTanque[0] = rect_points[0];
                   recTanque[1] = rect_points[1];
                   recTanque[2] = rect_points[3];
                   recTanque[3] = rect_points[2];
                 }
                 else if (rect_points[1].y>rect_points[3].y){

                   recTanque[0] = rect_points[2];
                   recTanque[1] = rect_points[3];
                   recTanque[2] = rect_points[1];
                   recTanque[3] = rect_points[0];
                 }
                 else {

                   recTanque[0] = rect_points[1];
                   recTanque[1] = rect_points[2];
                   recTanque[2] = rect_points[0];
                   recTanque[3] = rect_points[3];
                 }
               }
             }
           }
         }
         printf ("Tanque %d\n", indTanque);
         line(matretangulos, recTanque[0], recTanque[1], color, 3, 8 );
         line(matretangulos, recTanque[0], recTanque[2], color, 3, 8 );
         line(matretangulos, recTanque[2], recTanque[3], color, 3, 8 );
         line(matretangulos, recTanque[3], recTanque[1], color, 3, 8 );
         line(matretangulos, Point2f((recTanque[0].x-(recTanque[0].x-recTanque[1].x)*0.5), 0), Point2f((recTanque[0].x-(recTanque[0].x-recTanque[1].x)*0.5),frame.rows), color, 1, 8 );
         line(matretangulos, Point2f(frame.cols/2, 0), Point2f(frame.cols/2,frame.rows),  Scalar (0,255,0), 1, 8 );
         imshow( "Tanque", matretangulos);
         posicaoTanque ((recTanque[0].x-recTanque[1].x),  (recTanque[0].x-(recTanque[0].x-recTanque[1].x)*0.5), frame.cols/2);
         printf ("Ponto: %f de %d", (recTanque[0].x-(recTanque[0].x-recTanque[1].x)*0.5), frame.cols/2);
         printf ("Angulo: %lf\tDistancia: %f\n", angTanque, distTanque);
         if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

void posicaoTanque (float line1size,  float px1, float center){
  printf ("%f\n", line1size);
  float z1 = 412.251+1.6477*line1size+0.0018803*line1size*line1size;//-56/line1size;
  //float X1_CENTER_ERROR = X_ERROR_MI*z1 + X_ERROR_CONST;
  float x1 = (z1*(px1-center)/FOCAL_DIST)+X_ERROR_CONST;
  printf ("x: %f\n", x1);
  printf ("z: %f\n", z1);
  angTanque = atan (x1/z1)*360/3.141592;
  distTanque = pow ((pow (x1, 2) + pow (z1, 2)), 0.5);
}
