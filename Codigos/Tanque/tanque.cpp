#include "opencv2/opencv.hpp"
#include <stdlib.h>
#include <stdio.h>
using namespace cv;

int main(int, char**)
{
    VideoCapture cap("o3.avi"); // open the default camera
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
        inRange(equalizado,Scalar(20,150,140), Scalar(80,180,160),binario);

        imshow( "binario", binario);
        //printf ("3\n");
        vector<vector<Point> > contours;  //Para encontrar contornos
        vector<Vec4i> hierarchy;          //Para encontrar contornos

        findContours( binario, contours, hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

        //printf ("4\n");
        //Variaveis contorno
        vector<vector<Point> > contours_poly( contours.size());//Para encontrar contornos
        vector<RotatedRect>    minRect( contours.size());      //Para encontrar retangulo rotacionado


        //printf ("5\n");
        for( int i = 0; i < contours.size(); i++ ){                       //Para cada contorno
          approxPolyDP( Mat(contours[i]), contours_poly[i],5, true ); //Aproxima contorno por um poligono
          minRect[i] = minAreaRect( Mat(contours_poly[i]) );              //Menor retangulo rotacionado capaz de conter o contorno
         }

         //printf ("6\n");
         int maxArea = 0, indTanque;
         RNG rng (12345);
         Scalar color = Scalar (rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
         Point2f recTanque[4], rect_points[4];
         for( int i = 0; i< contours.size(); i++ ){
           //printf ("%d\n", i);
           minRect[i].points( rect_points );
           double line1, line2;
           line1 = norm (rect_points[0]-rect_points[1]);
           line2 = norm (rect_points[0]-rect_points[3]);
           if ((line1+line2)>(frame.rows+frame.cols)/50){
             double acontours = contourArea(contours_poly[i], false);
             if ((line1*line2/acontours)<1.4){
               if (acontours>maxArea){
                 maxArea = acontours;
                 indTanque = i;
                 if (rect_points[1].y>rect_points[3].y&&rect_points[2].y>rect_points[0].y){

                   recTanque[0] = rect_points[1];
                   recTanque[1] = rect_points[2];
                   recTanque[2] = rect_points[0];
                   recTanque[3] = rect_points[3];

                   //printf ("12\n");
                 }
                 else if (rect_points[0].y<rect_points[2].y){

                   recTanque[0] = rect_points[0];
                   recTanque[1] = rect_points[1];
                   recTanque[2] = rect_points[3];
                   recTanque[3] = rect_points[2];
                          //printf ("12\n");
                 }
                 else if (rect_points[1].y>rect_points[3].y){

                   recTanque[0] = rect_points[2];
                   recTanque[1] = rect_points[3];
                   recTanque[2] = rect_points[1];
                   recTanque[3] = rect_points[0];
                          printf ("12\n");
                 }
                 else {

                   recTanque[0] = rect_points[1];
                   recTanque[1] = rect_points[2];
                   recTanque[2] = rect_points[0];
                   recTanque[3] = rect_points[3];
                          //printf ("12\n");
                 }
               }
             }
           }
         }
         line(matretangulos, recTanque[0], recTanque[1], color, 3, 8 );
         line(matretangulos, recTanque[0], recTanque[2], color, 3, 8 );
         line(matretangulos, recTanque[2], recTanque[3], color, 3, 8 );
         line(matretangulos, recTanque[3], recTanque[1], color, 3, 8 );
         //printf ("13\n");
         imshow( "Tanque", matretangulos);

         //printf ("14\n");
         if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
