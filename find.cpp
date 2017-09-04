#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include<math.h>
#include <sys/time.h>

//Macros:
#define FOCAL_DIST 602.1197
#define X_ERROR_MI 0.129152		//The x axis position of the analised line have an error that grows linearly (approximately) with the distance in the z axis
#define X_ERROR_CONST -0.6332	//Mathematically: error = z1*X_ERROR_MI + X_ERROR_CONSTANT

using namespace cv;

Mat src, original, closing, matblur, gradient, binarizado, matcontornos, matretangulos, matprocessado, matxisdavaca, matretas, matfinal;
vector<float> vecz1, vecx1, vecang1, vecz2, vecx2, vecang2, vecdist;
Point2f pt1linha1(0.0,0.0), pt2linha1(0.0,0.0),pt1linha2(0.0,0.0), pt2linha2(0.0,0.0);
float z1, x1, ang1, z2, x2, ang2, dist;



int kblur = 5;
int ero1 = 4;
int ero2 = 3;
int razaoRetangulosVaca = 165;
int razaoRetangulosVaca2 = 170;
int poli = 5;
void savePoints ();
void CowRect(int, void*);
void position (float line1size, float line2size, float px1, float px2);
int main( int argc, char** argv ){
  //Calcula tempo de execução
  //CvCapture* cam;
  //cam = cvCreateCameraCapture( 1 );
  VideoCapture cam("/home/iduarte/Downloads/CowVid1.avi");
  //if(!cam) {printf("Impossível abrir camera\n"); return -1;}
  if(!cam.isOpened()) {printf("Impossível abrir camera\n"); return -1;}
  struct timeval tempo1, tempo2;
  int tempo, loop = 1;
  char tecla;
  cam >> src;
  closing = Mat::zeros(src.rows, src.cols, CV_8UC1);
  matblur = Mat::zeros(src.rows, src.cols, CV_8UC1);
  gradient = Mat::zeros(src.rows, src.cols, CV_8UC1);
  matfinal = Mat::zeros(src.rows, src.cols, CV_8UC1);

  while(loop)
	{
    gettimeofday (&tempo1, NULL);
  		//Captura frame
		//src = cvQueryFrame( cam );
    cam >> src;

    matprocessado =src.clone();
    matxisdavaca = src.clone();
    matretas = src.clone();
    binarizado = src.clone();
    matretangulos = src.clone();
      matcontornos = src.clone();

    if (src.empty())
      break;
		//Imprime captura na tela
    namedWindow ("Entrada", CV_WINDOW_NORMAL);
		imshow( "Entrada", src );

      /// Load an image
    original = src.clone();
    if( !src.data )
     return -1;

    //Equaliza cie ------------------------------
    Mat cie, cie2;
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
    cie2 = src.clone();
    namedWindow ("Equalizada", CV_WINDOW_NORMAL);
		imshow( "Equalizada", src );

    namedWindow( "Trackbar", CV_WINDOW_NORMAL );

    createTrackbar( "Blur (opcional):", "Trackbar", &kblur, 20, CowRect );
    createTrackbar( "Closing: ", "Trackbar", &ero1,  20,CowRect );
    createTrackbar( "Gradient: ", "Trackbar", &ero2,  20, CowRect );
    createTrackbar( "Razao largura/100: ", "Trackbar", &razaoRetangulosVaca, 300, CowRect );
    createTrackbar( "Razao altura/100: ", "Trackbar", &razaoRetangulosVaca2, 300, CowRect );
    createTrackbar( "poligono: ", "Trackbar", &poli, 50, CowRect );

    //Encontra retangulos da vaca
    CowRect(0, 0);


    //Calcula tempo de execução
    gettimeofday (&tempo2, NULL);
    tempo = (int)(1000000*(tempo2.tv_sec-tempo1.tv_sec)+(tempo2.tv_usec-tempo1.tv_usec));
    printf ("%d microseg\n%f Hezt\n", tempo, (1000000.0/tempo));

    tecla = cvWaitKey(15);
    if(tecla == 13){
      tecla = cvWaitKey(15);
      waitKey(0);
    }
    if (tecla == 's'){
      printf ("save\n");
        imwrite ("cena/equalizada.jpg" , cie2);
        imwrite ("cena/original.jpg" , original);
        imwrite ("cena/closing.jpg" ,closing);
        imwrite ("cena/matblur.jpg" ,matblur);
        imwrite ("cena/gradient.jpg" ,gradient);
        imwrite ("cena/binarizado.jpg" ,binarizado);
        imwrite ("cena/matcontornos.jpg" ,matcontornos);
        imwrite ("cena/matretangulos.jpg" ,matretangulos);
        imwrite ("cena/matprocessado.jpg" ,matprocessado);
        imwrite ("cena/matxisdavaca.jpg" ,matxisdavaca);
        imwrite ("cena/matretas.jpg" ,matretas);
        imwrite ("cena/matfinal.jpg" ,matfinal);
        waitKey(0);
    }
    if(tecla == 27)
      return 0;
  }

  waitKey(0);
  //savePoints ();
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
    closing = src2.clone();
    Mat copia = src2.clone();
    //Aplica blur se desejado
    if (kblur>0)
      blur(src2, src2, Size(kblur*2-1,kblur*2-1) );
    matblur = src2.clone();
    //Aplica filtro morphologico do tipo Gradient
    element=getStructuringElement(MORPH_RECT,Size(2*ero2+1,2*ero2+1),Point(ero2,ero2));
    morphologyEx(src2, binary, 4, element );
    gradient = binary.clone();
    //namedWindow ("gradient", CV_WINDOW_NORMAL);
    //imshow( "gradient", binary);


    cvtColor(binary,binary,CV_BGR2GRAY);
    threshold(binary, binary, 10, 255, THRESH_BINARY_INV);
    binarizado = binary.clone();
    //namedWindow ("Binarizada", CV_WINDOW_NORMAL);
    //imshow( "Binarizada", binary);

    //contornos
    findContours( binary, contours, hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
    //Variaveis contorno
    vector<vector<Point> > contours_poly( contours.size());//Para encontrar contornos
    vector<RotatedRect>    minRect( contours.size());//Para encontrar retangulo rotacionado

    //Para cada contorno
    for( int i = 0; i < contours.size(); i++ ){
      //Aproxima contorno por um poligono
      approxPolyDP( Mat(contours[i]), contours_poly[i], poli, true );
      //Menor retangulo rotacionado capaz de conter o contorno
      minRect[i] = minAreaRect( Mat(contours_poly[i]) );
     }
     RNG rng (12345);

     Mat tempBlackWhite = original.clone();
     double maxP = 0;
     for( int i = 0; i< contours.size(); i++ )
        {

          Scalar color = Scalar (rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
          //Recupera vertices do retangulo rotacionado
          Point2f rect_points[4]; minRect[i].points( rect_points );
          //drawContours (tempBlackWhite, contours, i,color, 1,8, hierarchy,0, Point ());
          //color = Scalar (rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
          //drawContours (tempBlackWhite, contours_poly, i,color, 1,8, hierarchy,0, Point ());
          drawContours (matcontornos, contours_poly, i,color, 1,8, hierarchy,0, Point ());


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
          if (rl<4&&(line1+line2)>(src.rows+src.cols)/50){
            //Calcula perimetro do contorno
            double pcontours = arcLength(contours_poly[i], 1);

            //Se perimetro do contorno for menor que 250% do perimetro do retangulo
            if (pcontours<3*(line1+line2)){
              //Calcula area do retangulo rotacionado
              double arec = line1*line2;
              //Calcula area do contorno
              double acontours = contourArea(contours_poly[i], false);

              //Se area do contorno for maior que ~83% do perimetro do retangulo
              if ((arec/acontours)<1.4){
                //Contador zerado
                hit.push_back(0);

                //Offset dos vertices, pois gradiente tente a diminuir tamanho real do retangulo.
                float compensa;
                if (kblur>0){
                  compensa = float (ero2+kblur*0.8);
                }
                else{
                  compensa = ero2;
                }
                Point2f p1(float (compensa), float (-compensa)), p2(float (compensa), float (compensa*1.0));
                Point2f p4(float (-compensa), float (-compensa)), p3(float (-compensa), float (compensa*1.0));

                //Encontra vertices
                if (rect_points[1].y>rect_points[3].y&&rect_points[2].y>rect_points[0].y){
                  line(matretangulos, rect_points[1], rect_points[3], color, 1, 8 );
                  line(matretangulos, rect_points[0], rect_points[2], color, 1, 8 );
                   vie.push_back(rect_points[0]+p3);
                   vse.push_back(rect_points[1]+p4);
                   vsd.push_back(rect_points[2]+p1);
                   vid.push_back(rect_points[3]+p2);
                }
                else if (rect_points[0].y<rect_points[2].y){
                  line(matretangulos, rect_points[1], rect_points[3], color, 1, 8 );
                line(matretangulos, rect_points[0], rect_points[2], color, 1, 8 );
                  vse.push_back(rect_points[0]+p4);
                  vsd.push_back(rect_points[1]+p1);
                  vid.push_back(rect_points[2]+p2);
                  vie.push_back(rect_points[3]+p3);
                }
                else if (rect_points[1].y>rect_points[3].y){
                  line(matretangulos, rect_points[1], rect_points[3], color, 1, 8 );
                line(matretangulos, rect_points[0], rect_points[2], color, 1, 8 );
                  vid.push_back(rect_points[0]+p2);
                  vie.push_back(rect_points[1]+p3);
                  vse.push_back(rect_points[2]+p4);
                  vsd.push_back(rect_points[3]+p1);
                }
                else {
                  line(matretangulos, rect_points[1], rect_points[3], color, 1, 8 );
                line(matretangulos, rect_points[0], rect_points[2], color, 1, 8 );
                  vie.push_back(rect_points[0]+p3);
                  vse.push_back(rect_points[1]+p4);
                  vsd.push_back(rect_points[2]+p1);
                  vid.push_back(rect_points[3]+p2);
                }
              }
            }
          }
        }
     //*Desenha 'X' vermelho nos retangulos
     for (int k = 0; k<vsd.size() ; k++){
       Scalar color = Scalar( 0, 100, 150);
       line(matprocessado, vsd[k], vie[k], color, 1, 8 );
       line(matprocessado, vse[k], vid[k], color, 1, 8 );
     }
     //*/
     int marcador, max = 0;
     float m, a, largura, altura, ysup, mediaMi, mmax, tipo;
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

       Vec3b pixel = copia.at<Vec3b>(pt.y, pt.x);
       int cor = pixel[0]+pixel[1]+pixel[2];
       pixel = copia.at<Vec3b>(pt.y, pt.x+largura);
       int cor2 = pixel[0]+pixel[1]+pixel[2];
       if (cor>cor2){
          tipo = 5.5;
       }
       else{
         tipo = 4.5;
       }
       //Calcula largura da vaca, se K for um retangulo da vaca
       double deltaX = tipo*largura+vsd[k].x;

       //Compara retangulo k, com todos os outros retangulos
       for (int l = 0; l<vsd.size() ; l++){
         //Calcula largura do retangulo L
         double largura2 = norm(vie[l]-vid[l]), rlar, ralt, altura2 = norm(vie[k]-vid[k]);

         //Maior razão entre largura dos retangulos K e L
         (largura2>largura)?rlar = largura2/largura:rlar = largura/largura2;
         (altura2>altura)  ?ralt = altura2/altura  :ralt = altura/altura2;
         //Para diminuir tempo de execução verifica se retangulos tem largura muito diferente
         //Se razão entre largura dos retangulos K e L for menor que valor de trackbar - default: 1.3 (130%)
         if (rlar<razaoRetangulosVaca/100.0&&ralt<razaoRetangulosVaca2/100.0){
           //Com a equação da reta do vertices do retangulo K
           //Calcula ysup dado a posição x do retangulo L
           ysup = m*vsd[l].x+a;

           //Verifica retangulo L pertence a vaca, considerando a hipotese que o retangulo K é da vaca
           if (vsd[l].x>=vsd[k].x //Se retangulo l esta a direita de k
             &&vsd[l].x<deltaX   //E dentro da largura da vaca
             &&vsd[l].y>ysup-10  //E se retangulo l esta dentro da altura maxima da vaca
             &&vsd[l].y<ysup+1.3*altura){ //E se retangulo l esta dentro da altura minima da vaca

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
      Scalar color = Scalar (rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
      line(matxisdavaca, vsd[marcador], vie[marcador], color, 1, 8 );
      line(matxisdavaca, vse[marcador], vid[marcador], color, 1, 8 );

      m = (vse[marcador].y-vsd[marcador].y)/(vse[marcador].x-vsd[marcador].x);
      //Calcula coeficiente linear do retangulo K
      a = vsd[marcador].y-vsd[marcador].x*m;

      //Calcula largura do retangulo K
      largura = norm(vie[marcador]-vid[marcador]),altura = norm(vsd[marcador]-vid[marcador]);

      pt.x = (vse[marcador].x + largura/2);
      pt.y = (vse[marcador].y+ altura/2);
      Vec3b pixel = copia.at<Vec3b>(pt.y, pt.x);
      int cor = pixel[0]+pixel[1]+pixel[2];
      pixel = copia.at<Vec3b>(pt.y, pt.x+largura);
      int cor2 = pixel[0]+pixel[1]+pixel[2];
      //printf ("x: %f\ty: %f\tcor: %d\n", pt.y, pt.x, cor);
      //printf ("0: %d\t1: %d\t2: %d\t", pixel[0], pixel[1], pixel[2]);
      //imshow ("copia", copia);
      float comparacao;
      if (cor>cor2){
         tipo = 5;
         comparacao = 3*largura+vsd[marcador].x;
      }
      else{
        comparacao = 3.4*largura+vse[marcador].x;
        tipo = 4;
      }
      double deltaX = tipo*largura+vsd[marcador].x;
/*
      Point2f aloca(comparacao, vsd[marcador].y);
      circle (tempBlackWhite, aloca, 10, Scalar (0, 0, 255),1,8,0);
      Point2f aloca2((tipo-1.5)*largura+vsd[marcador].x, vsd[marcador].y);
      circle (tempBlackWhite, aloca2, 10, Scalar (250, 250, 25),1,8,0);
//*/
      int minmarcador = marcador;
      float mindiferenca = 10000;

      ysup = m*vsd[marcador].x+a;

      Point2f alpha, beta;
      alpha.x = deltaX;
      alpha.y = ysup;
      beta.x = vse[marcador].x;
      beta.y = float (ysup+altura*1.3);

      //line(tempBlackWhite, vse[marcador], alpha, Scalar (255, 0, 255), 1, 8 );
      //line(tempBlackWhite, vse[marcador], beta, Scalar (255, 0, 255), 1, 8 );

      int contador = 1, supcontador = 1;
      float sumx, sumxsq, sumy, sumxy, a0, a1, denom, supsumx, supsumxsq, supsumy, supsumxy, supa0, supa1;
      supsumx = vsd[marcador].x;
      supsumxsq = pow(vsd[marcador].x, 2);
      supsumy = vsd[marcador].y;
      supsumxy = vsd[marcador].x * vsd[marcador].y;
      sumx = vid[marcador].x;
      sumxsq = pow(vid[marcador].x, 2);
      sumy = vid[marcador].y;
      sumxy = vid[marcador].x * vid[marcador].y;


      for (int l = 0; l<vsd.size() ; l++){
        //Calcula largura do retangulo L
        double largura2 = norm(vie[l]-vid[l]), rlar, altura2 = norm(vie[l]-vid[l]), ralt;

        //Maior razão entre largura dos retangulos K e L
        (largura2>largura)?rlar = largura2/largura:rlar = largura/largura2;

        (altura2>altura)  ?ralt = altura2/altura  :ralt = altura/altura2;
        //Para diminuir tempo de execução verifica se retangulos tem largura muito diferente
        //Se razão entre largura dos retangulos K e L for menor que valor de trackbar - default: 1.3 (130%)
        if (rlar<razaoRetangulosVaca/100.0&&ralt<razaoRetangulosVaca2/100.0){
          //Com a equação da reta do vertices do retangulo K
          //Calcula ysup dado a posição x do retangulo L
          ysup = m*vsd[l].x+a;

          //Verifica retangulo L pertence a vaca, considerando a hipotese que o retangulo K é da vaca
          if (vsd[l].x>=vsd[marcador].x //Se retangulo l esta a direita de k
            &&vsd[l].x<deltaX   //E dentro da largura da vaca
            &&vsd[l].y>ysup-15  //E se retangulo l esta dentro da altura maxima da vaca
            &&vsd[l].y<ysup+1.3*altura){ //E se retangulo l esta dentro da altura minima da vaca
              if (abs(comparacao-vsd[l].x)<mindiferenca){
                mindiferenca = abs(deltaX-largura-vsd[l].x);
                minmarcador = l;
              }
              if (vse[l].y*1.3<vid[marcador].y){
                sumx += vid[l].x;
                sumxsq += pow(vid[l].x, 2);
                sumy += vid[l].y;
                sumxy += vid[l].x * vid[l].y;

                supsumx += vsd[l].x;
                supsumxsq += pow(vsd[l].x, 2);
                supsumy += vsd[l].y;
                supsumxy += vsd[l].x * vsd[l].y;
                supcontador++;
              }
              else {
                sumx += vsd[l].x;
                sumxsq += pow(vsd[l].x, 2);
                sumy += vsd[l].y;
                sumxy += vsd[l].x * vsd[l].y;
              }
              line(matxisdavaca, vsd[l], vie[l], color, 1, 8 );
              line(matxisdavaca, vse[l], vid[l], color, 1, 8 );
              contador++;
  //          line(tempBlackWhite, vse[l], vid[l], Scalar (150, 100, 0), 1, 8 );
  //          line(tempBlackWhite, vie[l], vsd[l], Scalar (150, 100, 0), 1, 8 );
          }
        }
      }
      denom = contador * sumxsq - pow(sumx, 2);
      a1 = (contador * sumxy - sumx*sumy)/denom;
      a0 = (sumy - a1*sumx)/contador;

      denom = supcontador * supsumxsq - pow(supsumx, 2);
      supa1 = (supcontador * supsumxy - supsumx*supsumy)/denom;
      supa0 = (supsumy - supa1*supsumx)/supcontador;
//*

      float porcentagem = 0.8;

      if (tipo==5){
        pt1linha1.x = pt1linha1.x*(1-porcentagem)+porcentagem*vsd[marcador].x;
        pt1linha1.y = pt1linha1.y*(1-porcentagem)+porcentagem*(vsd[marcador].x*supa1+supa0);
        pt2linha1.x = pt2linha1.x*(1-porcentagem)+porcentagem*vsd[marcador].x;;
        pt2linha1.y = pt2linha1.y*(1-porcentagem)+porcentagem*(vsd[marcador].x*a1+a0);
        if (pt1linha1.x!=pt1linha1.x)
          pt1linha1.x = vsd[marcador].x;
        if (pt1linha1.y!=pt1linha1.y)
          pt1linha1.y = vsd[marcador].x*supa1+supa0;
        if (pt2linha1.x!=pt2linha1.x)
          pt2linha1.x = vsd[marcador].x;
        if (pt2linha1.y!=pt2linha1.y)
          pt2linha1.y = vsd[marcador].x*a1+a0;
      }
      else{
        pt1linha1.x = pt1linha1.x*(1-porcentagem)+porcentagem*vse[marcador].x;
        pt1linha1.y = pt1linha1.y*(1-porcentagem)+porcentagem*(vse[marcador].x*supa1+supa0);
        pt2linha1.x = pt2linha1.x*(1-porcentagem)+porcentagem*vse[marcador].x;;
        pt2linha1.y = pt2linha1.y*(1-porcentagem)+porcentagem*(vse[marcador].x*a1+a0);
        if (pt1linha1.x!=pt1linha1.x)
          pt1linha1.x = vse[marcador].x;
        if (pt1linha1.y!=pt1linha1.y)
          pt1linha1.y = vse[marcador].x*supa1+supa0;
        if (pt2linha1.x!=pt2linha1.x)
          pt2linha1.x = vse[marcador].x;
        if (pt2linha1.y!=pt2linha1.y)
          pt2linha1.y = vse[marcador].x*a1+a0;
      }

      line(tempBlackWhite, pt1linha1, pt2linha1, Scalar (0, 20, 255), 2, 8 );

      pt1linha2.x = pt1linha2.x*(1-porcentagem)+porcentagem*vsd[minmarcador].x;
      pt1linha2.y = pt1linha2.y*(1-porcentagem)+porcentagem*(vsd[minmarcador].x*supa1+supa0);
      pt2linha2.x = pt2linha2.x*(1-porcentagem)+porcentagem*vsd[minmarcador].x;
      pt2linha2.y = pt2linha2.y*(1-porcentagem)+porcentagem*(vsd[minmarcador].x*a1+a0);
      if (pt1linha2.x!=pt1linha2.x)
        pt1linha2.x = vsd[minmarcador].x;
      if (pt1linha2.y!=pt1linha2.y)
        pt1linha2.y = vsd[minmarcador].x*supa1+supa0;
      if (pt2linha2.x!=pt2linha2.x)
        pt2linha2.x = vsd[minmarcador].x;
      if (pt2linha2.y!=pt2linha2.y)
      pt2linha2.y = vsd[minmarcador].x*a1+a0;
      line(tempBlackWhite, pt1linha2, pt2linha2, Scalar (0, 20, 255), 2, 8 );
      color = Scalar (rng.uniform(100,255),rng.uniform(100,255),rng.uniform(0,255));
      line(matretas, pt1linha1, pt1linha2, color, 1, 8 );
      line(matretas, pt2linha1, pt2linha2, color, 1, 8 );


      //printf ("p1x: %f\tp1y: %f\tp2x: %f\tp2x: %f\tp3x: %f\tp3y: %f\tp4x: %f\tp4y: %f\n",pt1linha1.x,pt1linha1.y,pt2linha1.x,pt2linha1.y,pt1linha2.x,pt1linha2.y,pt2linha2.x,pt2linha2.y);
      position (pt2linha1.y-pt1linha1.y, pt2linha2.y-pt1linha2.y, pt1linha1.x,pt1linha2.x);
      //printf ("z1: %f\tx1: %f\t ang1: %f\t z2: %f\t x2: %f\t ang2: %f\t dist: %f\n", z1, x1, ang1, z2, x2, ang2, dist);
      matfinal = tempBlackWhite.clone();

    }

    imshow( "Output", tempBlackWhite);
}

void position (float line1size, float line2size, float px1, float px2){
  float xcenter = src.cols/2;
  z1 = (FOCAL_DIST*10.0)/line1size;
	z2 = (FOCAL_DIST*10.0)/line2size;
	float X1_CENTER_ERROR = X_ERROR_MI*z1 + X_ERROR_CONST;
	float X2_CENTER_ERROR = X_ERROR_MI*z2 + X_ERROR_CONST;
	x1 = (z1*(px1-xcenter)/FOCAL_DIST)-X1_CENTER_ERROR;
	x2 = (z2*(px2-xcenter)/FOCAL_DIST)-X2_CENTER_ERROR;
	dist = pow( pow(z1-z2, 2) + pow(x1-x2, 2) , 0.5)+3.0;
	ang1 = tan(x1/z1);
  ang2 = tan(x2/z2);
  vecz1.push_back(z1);
  vecx1.push_back(x1);
  vecang1.push_back(ang1);
  vecz2.push_back(z2);
  vecx2.push_back(x2);
  vecang2.push_back(ang2);
  vecdist.push_back(dist);
}

void savePoints (){
  FILE *fp;
  fp = fopen("pontos.txt", "w");
  fprintf (fp, "vecz1\n");
  for (std::vector<float>::iterator it = vecz1.begin() ; it != vecz1.end(); ++it){
    fprintf (fp, "%f\n", *it);
  }
  fprintf (fp, "vecx1\n");
  for (std::vector<float>::iterator it =  vecx1.begin() ; it != vecx1.end(); ++it){
    fprintf (fp, "%f\n", *it);
  }
  fprintf (fp, "vecang1\n");
  for (std::vector<float>::iterator it = vecang1.begin() ; it != vecang1.end(); ++it){
    fprintf (fp, "%f\n", *it);
  }
  fprintf (fp, "vecz2\n");
  for (std::vector<float>::iterator it =  vecz2.begin() ; it != vecz2.end(); ++it){
    fprintf (fp, "%f\n", *it);
  }
  fprintf (fp, "vecx2\n");
  for (std::vector<float>::iterator it = vecx2.begin() ; it != vecx2.end(); ++it){
    fprintf (fp, "%f\n", *it);
  }
  fprintf (fp, "vecang2\n");
  for (std::vector<float>::iterator it = vecang2.begin() ; it != vecang2.end(); ++it){
    fprintf (fp, "%f\n", *it);
  }
  fprintf (fp, "vecdist\n");
  for (std::vector<float>::iterator it = vecdist.begin() ; it != vecdist.end(); ++it){
    fprintf (fp, "%f\n", *it);
  }
  fclose (fp);
}
