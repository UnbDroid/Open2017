#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>

using namespace cv;

Mat src, original;

int kblur = 0;
int ero1 = 5;
int ero2 = 3;
int razaoRetangulosVaca = 13;

void CowRect(int, void*);

int main( int argc, char** argv ){
  //Calcula tempo de execução
  struct timeval tempo1, tempo2;
  int tempo;
  gettimeofday (&tempo1, NULL);

  //Abre imagem
  src = imread( argv[1] );
  original = src.clone();
  if( !src.data )
   return -1;

  imshow ("Original", src);

  //Equaliza cie ------------------------------
  Mat cie;
  //Converte para Cie L*a*b
  cvtColor(src,cie,CV_BGR2Lab);
  vector<Mat> channels2;
  //Separa canais da imagem
  split(cie,channels2);
  //Equaliza canal L
  equalizeHist(channels2[0],channels2[0]);
  //Reune canais novamente
  merge(channels2,cie);
  //Converte para BGR
  cvtColor(cie,src,CV_Lab2BGR);

  namedWindow( "Trackbar", CV_WINDOW_NORMAL );

  createTrackbar( "Blur (opcional):", "Trackbar", &kblur, 20, CowRect );
  createTrackbar( "Closing: ", "Trackbar", &ero1,  20,CowRect );
  createTrackbar( "Gradient: ", "Trackbar", &ero2,  20, CowRect );
  createTrackbar( "Razao retangulo: ", "Trackbar", &razaoRetangulosVaca, 300, CowRect );

  //Encontra retangulos da vaca
  CowRect(0, 0);


  //Calcula tempo de execução
  gettimeofday (&tempo2, NULL);
  tempo = (int)(1000000*(tempo2.tv_sec-tempo1.tv_sec)+(tempo2.tv_usec-tempo1.tv_usec));
  printf ("%d microseg\n%f Hezt\n", tempo, (1000000.0/tempo));


  waitKey(0);
  return 0;
}

  void CowRect(int, void*)
  {

    vector<Point2f> vsd,vid,vie,vse;//Cria vetores para guardar melhores retangulos
    vector<int> hit; //Contador
    vector<vector<Point> > contours;//, contoursBlack; //Para encontrar contornos
    vector<Vec4i> hierarchy; //Para encontrar contornos

    //Aplica filtro morphologico do tipo closing
    Mat src2, binary, element=getStructuringElement(MORPH_RECT,Size(2*ero1+1,2*ero1+1),Point(ero1,ero1));
    morphologyEx(src, src2, 3, element );

    //Aplica blur se desejado
    if (kblur>0)
      blur(src2, src2, Size(kblur*2-1,kblur*2-1) );

    //Aplica filtro morphologico do tipo Gradient
    element=getStructuringElement(MORPH_RECT,Size(2*ero2+1,2*ero2+1),Point(ero2,ero2));
    morphologyEx(src2, binary, 4, element );
    cvtColor(binary,binary,CV_BGR2GRAY);
    threshold(binary, binary, 10, 255, THRESH_BINARY_INV);

    //contornos
    findContours( binary, contours, hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

    //Variaveis contorno
    vector<vector<Point> > contours_poly( contours.size());//Para encontrar contornos
    vector<RotatedRect>    minRect( contours.size());//Para encontrar retangulo rotacionado

    //Para cada contorno
    for( int i = 0; i < contours.size(); i++ ){
      //Aproxima contorno por um poligono
      approxPolyDP( Mat(contours[i]), contours_poly[i], 5, true );
      //Menor retangulo rotacionado capaz de conter o contorno
      minRect[i] = minAreaRect( Mat(contours_poly[i]) );
     }

     Mat tempBlackWhite = original.clone();
     double maxP = 0;
     for( int i = 0; i< contours.size(); i++ )
        {
          //Recupera vertices do retangulo rotacionado
          Point2f rect_points[4]; minRect[i].points( rect_points );

          double line1, line2;

          //Calcula base a altura do retangulo
          line1 = norm (rect_points[0]-rect_points[1]);
          line2 = norm (rect_points[0]-rect_points[3]);

          //Razão entre largura e altura do retangulo
          double rl;
          //condição ?  Verdadeiro      : Falso
          line1>line2?rl = (line1/line2):rl = (line2/line1);

          //Razão entre largura e altura do retangulo não pode ser maior que 3.5, isto implica
          //que retangulos esbeltos são rejeitados e retangulos muito pequenos também
          if (rl<3.5&&(line1+line2)>(src.rows+src.cols)/35){
            //Calcula perimetro do contorno
            double pcontours = arcLength(contours_poly[i], 1);

            //Se perimetro do contorno for menor que 250% do perimetro do retangulo
            if (pcontours<2*(line1+line2)){
              //Calcula area do retangulo rotacionado
              double arec = line1*line2;
              //Calcula area do contorno
              double acontours = contourArea(contours_poly[i], false);

              //Se area do contorno for maior que ~83% do perimetro do retangulo
              if ((arec/acontours)<1.2){
                //Contador zerado
                hit.push_back(0);

                //Offset dos vertices, pois gradiente tente a diminuir tamanho real do retangulo.
                Point2f p1(float (ero2), float (-ero2)), p2(float (ero2), float (ero2*1.0));
                Point2f p4(float (-ero2), float (-ero2)), p3(float (-ero2), float (ero2*1.0));

                //Encontra vertices
                if (rect_points[1].y>rect_points[3].y&&rect_points[2].y>rect_points[0].y){
                   vie.push_back(rect_points[0]+p3);
                   vse.push_back(rect_points[1]+p4);
                   vsd.push_back(rect_points[2]+p1);
                   vid.push_back(rect_points[3]+p2);
                }
                else if (rect_points[0].y<rect_points[2].y){
                  vse.push_back(rect_points[0]+p4);
                  vsd.push_back(rect_points[1]+p1);
                  vid.push_back(rect_points[2]+p2);
                  vie.push_back(rect_points[3]+p3);
                }
                else if (rect_points[1].y>rect_points[3].y){
                  vid.push_back(rect_points[0]+p2);
                  vie.push_back(rect_points[1]+p3);
                  vse.push_back(rect_points[2]+p4);
                  vsd.push_back(rect_points[3]+p1);
                }
                else {
                  vie.push_back(rect_points[0]+p3);
                  vse.push_back(rect_points[1]+p4);
                  vsd.push_back(rect_points[2]+p1);
                  vid.push_back(rect_points[3]+p2);
                }
              }
            }
          }
        }

     //Desenha 'X' vermelho nos retangulos
     for (int k = 0; k<vsd.size() ; k++){
       Scalar color = Scalar( 0, 0, 255);
       line(tempBlackWhite, vsd[k], vie[k], color, 1, 8 );
       line(tempBlackWhite, vse[k], vid[k], color, 1, 8 );
     }

     int marcador, max = 0;
     double m, a, largura, altura, ysup, mediaMi, mmax;
     Point2f pt;

//####################################################################################################
//####################################################################################################
//PARTE DO CODIGO PASSIVEL DE MELHORIAS APARTIR DAQUI
//####################################################################################################
//####################################################################################################
     /*
     Para cada retangulo encontrado, verifica se ele é o retangulo superior a esquerda da lateral da vaca
     para tal cria um caixa como se fosse a vaca e conta quantos retangulos tem dentro o que tiver mais
     é o melhor canditado
     */
     for (int k = 0; k<vsd.size() ; k++){
       //Calcula coeficiente angular do retangulo K
       m = (vse[k].y-vsd[k].y)/(vse[k].x-vsd[k].x);
       mediaMi = m;

       //Calcula coeficiente linear do retangulo K
       a = vsd[k].y-vsd[k].x*m;

       //Calcula largura do retangulo K
       largura = norm(vie[k]-vid[k]), altura = norm(vsd[k]-vid[k]);

       //Calcula largura da vaca, se K for um retangulo da vaca
       double deltaX = 6*largura+vsd[k].x;

       //Compara retangulo k, com todos os outros retangulos
       for (int l = 0; l<vsd.size() ; l++){
         //Calcula largura do retangulo L
         double largura2 = norm(vie[k]-vid[k]), rlar;

         //Maior razão entre largura dos retangulos K e L
         (largura2>largura)?rlar = largura2/largura:rlar = largura/largura2;

         //Para diminuir tempo de execução verifica se retangulos tem largura muito diferente
         //Se razão entre largura dos retangulos K e L for menor que valor de trackbar - default: 1.3 (130%)
         if (rlar<razaoRetangulosVaca/10.0){
           //Com a equação da reta do vertices do retangulo K
           //Calcula ysup dado a posição x do retangulo L
           ysup = m*vsd[l].x+a;

           //Verifica retangulo L pertence a vaca, considerando a hipotese que o retangulo K é da vaca
           if (vsd[l].x>=vsd[k].x //Se retangulo l esta a direita de k
             &&vsd[l].x<deltaX   //E dentro da largura da vaca
             &&vsd[l].y>ysup-10  //E se retangulo l esta dentro da altura maxima da vaca
             &&vsd[l].y<ysup+3*(altura/2)){ //E se retangulo l esta dentro da altura minima da vaca

               //Calcula coeficiente angular do retangulo L
               double m2 = (vse[l].y-vsd[l].y)/(vse[l].x-vsd[l].x);

               //Soma coeficientes angular do retangulo L e K
               mediaMi += m2;

               //Adiciona +1 no contador de retangulos que cumprem relações se pertecerem a vaca
               hit[k]+=1;
           }
         }
       }
       //Procura retangulos que mais tem relações com outros retangulos
       if (hit[k]>max){
         mmax = mediaMi/(hit[k]); //Coeficiente angular medio
         marcador = k; //Retangulo com mais pontos;
         max = hit[k]; //Pontos do melhor canditado;
       }
     }


    //Desenha
    Point2f pt2;
    if (max>3){
      //Desenha X azul nos quadrados dentro da vaca
      for (int j=0;j<max;j++){
        line(tempBlackWhite, vse[quadradosDentro[marcador][j]], vid[quadradosDentro[marcador][j]], Scalar (255, 100, 0), 2, 8 );
        line(tempBlackWhite, vsd[quadradosDentro[marcador][j]], vie[quadradosDentro[marcador][j]], Scalar (255, 100, 0), 2, 8 );
      }

      //Desenha Retangulo superior direita da lateral da vaca
      line(tempBlackWhite, vse[marcador], vie[marcador], Scalar (255, 255, 0), 2, 8 );
      line(tempBlackWhite, vie[marcador], vid[marcador], Scalar (255, 255, 0), 2, 8 );
      line(tempBlackWhite, vid[marcador], vsd[marcador], Scalar (255, 255, 0), 2, 8 );
      line(tempBlackWhite, vsd[marcador], vse[marcador], Scalar (255, 255, 0), 2, 8 );

      //Calcula coeficiente linear da lateral da vaca
      a = vie[marcador].y-vie[marcador].x*mmax;

      //Calcula largura retangulo da vaca
      largura = norm(vie[marcador]-vid[marcador]);

      //fim x da vaca = 6*retangulo da vaca
      pt.x = float (6*largura+vie[marcador].x);
      //fim y da vaca = 6*retangulo da vaca
      pt.y = float (mmax*pt.x+a);

      //Desenha linha lateral da vaca
      line(tempBlackWhite, vie[marcador], pt, Scalar (0, 255, 0), 2, 8 );
    }
    imshow( "Output", tempBlackWhite);
}
