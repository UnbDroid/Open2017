#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <sys/time.h>

//Macros:
#define FOCAL_DIST 602.1197
#define X_ERROR_MI 0.129152		//The x axis position of the analised line have an error that grows linearly (approximately) with the distance in the z axis
#define X_ERROR_CONST -0.6332	//Mathematically: error = z1*X_ERROR_MI + X_ERROR_CONSTANT

using namespace cv;
using namespace std;

Mat src, original, matblur, gradient, matretangulos, matprocessado, matxisdavaca, matretas, matfinal, tempBlackWhite;
vector<float> vecz1, vecx1, vecang1, vecz2, vecx2, vecang2, vecdist;
Point2f pt1linha1(0.0,0.0), pt2linha1(0.0,0.0),pt1linha2(0.0,0.0), pt2linha2(0.0,0.0);
float z1, x1, ang1, z2, x2, ang2, dist, err;
float max_error = 0.2;

int frame_width;
int frame_height;


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
  VideoCapture cam ("CowVid1.avi");
  //VideoCapture cam (1);
  if(!cam.isOpened()) {printf("Impossível abrir camera\n"); return -1;}

  struct timeval tempo1, tempo2;
  int tempo, loop = 1;

  char tecla;
  cam >> src;

  frame_width =   src.cols;
  frame_height =   src.rows;

  VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);

  matblur = Mat::zeros(src.rows, src.cols, CV_8UC1);
  gradient = Mat::zeros(src.rows, src.cols, CV_8UC1);
  matfinal = Mat::zeros(src.rows, src.cols, CV_8UC1);

  while(loop)
	{
    gettimeofday (&tempo1, NULL);
  	//Captura frame
    cout << "Getting frame...\n";
    cam >> src;

    matprocessado = src.clone();
    matxisdavaca = src.clone();
    matretas = src.clone();
    matretangulos = src.clone();

    if(src.empty())
    {
      cout << "No more frames\n";
      break;
    }

    cout << "OK Got frame\n";

		//Imprime captura na tela
    
    cout << "Showing src at the 'Entrada' window...\n";
		imshow("Entrada", src);
    cout << "OK imshow worked\n";
    //Load an image
    original = src.clone();
    if( !src.data ) {return -1;}

    cout << "Equalizing image...\n";

    //Equaliza cie ------------------------------
    Mat cie, cie2;
    cvtColor(src,cie,CV_BGR2Lab);//Converte para Cie L*a*b
    vector<Mat> channels2;
    split(cie,channels2);//Separa canais da imagem
    equalizeHist(channels2[0],channels2[0]);//Equaliza canal L
    merge(channels2,cie);//Reune canais novamente

    //Converte para BGR
    cvtColor(cie,src,CV_Lab2BGR);
    cie2 = src.clone();

    cout << "OK Image is equalized\n";

    namedWindow( "Trackbar", CV_WINDOW_NORMAL );
    createTrackbar( "Blur (opcional):",     "Trackbar", &kblur,               20,  CowRect );
    createTrackbar( "Closing: ",            "Trackbar", &ero1,                20,  CowRect );
    createTrackbar( "Gradient: ",           "Trackbar", &ero2,                20,  CowRect );
    createTrackbar( "Razao largura/100: ",  "Trackbar", &razaoRetangulosVaca, 300, CowRect );
    createTrackbar( "Razao altura/100: ",   "Trackbar", &razaoRetangulosVaca2,300, CowRect );
    createTrackbar( "poligono: ",           "Trackbar", &poli,                50,  CowRect );

    cout << "Calling CowRect function\n";

    //Encontra retangulos da vaca
    CowRect(0, 0);

    cout << "OK CowRect went through\n";

    //Calcula tempo de execução
    gettimeofday (&tempo2, NULL);
    tempo = (int)(1000000*(tempo2.tv_sec-tempo1.tv_sec)+(tempo2.tv_usec-tempo1.tv_usec));
    printf ("%d microseg\n%f Hertz\n", tempo, (1000000.0/tempo));

    cout << "WaitKey moment\n";

    tecla = waitKey(10);
    if(tecla == 27)
    {
      return 0;
    }

    cout << "End of WaitKey moment\n";
  }

  //waitKey(0);
  savePoints ();
  return 0;
}

void CowRect(int, void*)
{
  vector<Point2f> vsd,vid,vie,vse;  //Cria vetores para guardar melhores retangulos
  vector<int> hit, retangulosVaca, tempInt;                  //Contador
  vector<vector<Point> > contours;  //Para encontrar contornos
  vector<Vec4i> hierarchy;          //Para encontrar contornos

  //Aplica filtro morphologico do tipo closing
  Mat processando, src2, binary, element=getStructuringElement(MORPH_RECT,Size(2*ero1+1,2*ero1+1),Point(ero1,ero1));
  morphologyEx(src, src2, 3, element );

  Mat copiaEqualizada = src2.clone();

  //Aplica blur se desejado
  if (kblur>0)
    blur(src2, processando, Size(kblur*2-1,kblur*2-1));

  //Aplica filtro morphologico do tipo Gradient
  element=getStructuringElement(MORPH_RECT,Size(2*ero2+1,2*ero2+1),Point(ero2,ero2));
  morphologyEx(processando, binary, 4, element );

  //Binariza
  cvtColor(binary,processando,CV_BGR2GRAY);
  threshold(processando, binary, 10, 255, THRESH_BINARY_INV);

  //contornos
  findContours( binary, contours, hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

  //Variaveis contorno
  vector<vector<Point> > contours_poly( contours.size());//Para encontrar contornos
  vector<RotatedRect>    minRect( contours.size());      //Para encontrar retangulo rotacionado


  for( int i = 0; i < contours.size(); i++ ){                       //Para cada contorno
    approxPolyDP( Mat(contours[i]), contours_poly[i], poli, true ); //Aproxima contorno por um poligono
    minRect[i] = minAreaRect( Mat(contours_poly[i]) );              //Menor retangulo rotacionado capaz de conter o contorno
   }

   RNG rng (12345);

   tempBlackWhite = original.clone();
   double maxP = 0;
   for( int i = 0; i< contours.size(); i++ ){
        Scalar color = Scalar (rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255)); //Cor aleatoria
        Point2f rect_points[4]; minRect[i].points( rect_points );//Recupera vertices do retangulo rotacionado

        //Calcula base a altura do retangulo
        double line1, line2;
        line1 = norm (rect_points[0]-rect_points[1]);
        line2 = norm (rect_points[0]-rect_points[3]);

        //Razão entre largura e altura do retangulo
        double rl;
        //condição ?  Verdadeiro      : Falso
        line1>line2?rl = (line1/line2):rl = (line2/line1);

        //Razão entre largura e altura do retangulo não pode ser maior que 4 (isto implica
        //que retangulos esbeltos são rejeitados) e retangulos não devem ser muito pequenos
        if (rl<4&&(line1+line2)>(src.rows+src.cols)/50){
          double pcontours = arcLength(contours_poly[i], 1);//Calcula perimetro do contorno

          if (pcontours<3*(line1+line2)){//Se perimetro do contorno for menor que 300% do perimetro do retangulo
            double arec = line1*line2;  //Calcula area do retangulo rotacionado
            double acontours = contourArea(contours_poly[i], false);//Calcula area do contorno

            //Se area do contorno for maior que ~83% do perimetro do retangulo
            if ((arec/acontours)<1.4){
              hit.push_back(0); //Contador zerado

              //Offset dos vertices, pois gradiente tente a diminuir tamanho real do retangulo.
              float compensa;
              //condição?        Se Verdadeiro            : Se Falso
              kblur>0   ?compensa = float (ero2+kblur*0.8):compensa = ero2;
              Point2f p1(float (compensa), float (-compensa)), p2(float (compensa), float (compensa*1.0));
              Point2f p4(float (-compensa), float (-compensa)), p3(float (-compensa), float (compensa*1.0));

              //Encontra vertices, compensando rotação
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
   //*
   for (int k = 0; k<vsd.size() ; k++){
     Scalar color = Scalar( 0, 100, 150);
     line(matprocessado, vsd[k], vie[k], color, 1, 8 );
     line(matprocessado, vse[k], vid[k], color, 1, 8 );
   }
   namedWindow ("Quadrados", CV_WINDOW_NORMAL);
   imshow( "Quadrados", matprocessado);
   //*/

   int marcador, max = 0;
   float m, a, largura, altura, ysup, mediaMi, mmax, tipo;
   Point2f pt;

   /*
   Para cada retangulo encontrado, verifica se ele é o retangulo superior a esquerda da lateral da vaca
   para tal cria um caixa como se fosse a vaca e conta quantos retangulos tem dentro o que tiver mais
   é o melhor canditado
   */
   for (int k = 0; k<vsd.size() ; k++){
     m = (vse[k].y-vsd[k].y)/(vse[k].x-vsd[k].x);//Calcula coeficiente angular do retangulo K
     mediaMi = m; //Faz somatório para tirar média
     a = vsd[k].y-vsd[k].x*m;//Calcula coeficiente linear do retangulo K
     largura = norm(vie[k]-vid[k]), altura = norm(vsd[k]-vid[k]);//Calcula largura do retangulo K

     //Verifica se o retangulo é branco ou preto
     Vec3b pixel = copiaEqualizada.at<Vec3b>(pt.y, pt.x);
     int cor = pixel[0]+pixel[1]+pixel[2];
     pixel = copiaEqualizada.at<Vec3b>(pt.y, pt.x+largura);
     int cor2 = pixel[0]+pixel[1]+pixel[2];
     cor>cor2?tipo = 5.5:tipo = 4.5;

     double deltaX = tipo*largura+vsd[k].x;//Calcula largura da vaca, se K for um retangulo da vaca

     for (int l = 0; l<vsd.size() ; l++){ //Compara retangulo k, com todos os outros retangulos
       double largura2 = norm(vie[l]-vid[l]), rlar, ralt, altura2 = norm(vie[k]-vid[k]);//Calcula largura do retangulo L

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

             double m2 = (vse[l].y-vsd[l].y)/(vse[l].x-vsd[l].x);//Calcula coeficiente angular do retangulo L
             mediaMi += m2; //Soma coeficientes angular do retangulo L e K
             hit[k]+=1;     //Adiciona +1 no contador de retangulos que cumprem relações se pertecerem a vaca
             tempInt.push_back (l);//Armazena retangulos que deram hit
         }
       }
     }
     //Procura retangulos que mais tem relações com outros retangulos
     if (hit[k]>max){
       retangulosVaca = tempInt; //Retangulos da vaca
       tempInt.clear();
       mmax = mediaMi/(hit[k]); //Coeficiente angular medio
       marcador = k; //Retangulo com mais pontos;
       max = hit[k]; //Pontos do melhor canditado;
     }
   }

  //Desenha
  Point2f pt2;
  if (max>3){
    Scalar color = Scalar (rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));

    m = (vse[marcador].y-vsd[marcador].y)/(vse[marcador].x-vsd[marcador].x);//Calcula coeficiente angular do retangulo K
    a = vsd[marcador].y-vsd[marcador].x*m;//Calcula coeficiente linear do retangulo K
    largura = norm(vie[marcador]-vid[marcador]),altura = norm(vsd[marcador]-vid[marcador]);//Calcula largura do retangulo K


    //Verifica se primeiro quadrado é branco ou preto
    pt.x = (vse[marcador].x + largura/2);
    pt.y = (vse[marcador].y+ altura/2);
    Vec3b pixel = copiaEqualizada.at<Vec3b>(pt.y, pt.x);
    int cor = pixel[0]+pixel[1]+pixel[2];
    pixel = copiaEqualizada.at<Vec3b>(pt.y, pt.x+largura);
    int cor2 = pixel[0]+pixel[1]+pixel[2];
    float comparacao;
    if (cor>cor2){ //quadrado branco
      tipo = 5;
      comparacao = 3*largura+vsd[marcador].x;  }
    else{ //quadrado preto
      comparacao = 3.4*largura+vse[marcador].x;
      tipo = 4;
    }

    double deltaX = tipo*largura+vsd[marcador].x;

    int minmarcador = marcador;
    float mindiferenca = 10000;

    ysup = m*vsd[marcador].x+a;

    int contador = 1, supcontador = 1;
    float sumx, sumxsq, sumy, sumxy, a0, a1, denom, supsumx, supsumxsq, supsumy, supsumxy, supa0, supa1;

    supsumx = vsd[marcador].x;
    supsumxsq = pow(vsd[marcador].x, 2);
    supsumy = vsd[marcador].y;
    supsumxy = vsd[marcador].x * vsd[marcador].y;

    for (int l = 0; l<retangulosVaca.size() ; l++){
        line(tempBlackWhite, vsd[retangulosVaca[l]], vid[retangulosVaca[l]], Scalar (55, 250, 25), 1, 8 );
        if (abs(comparacao-vsd[retangulosVaca[l]].x)<mindiferenca){
          mindiferenca = abs(deltaX-largura-vsd[retangulosVaca[l]].x);
          minmarcador = retangulosVaca[l];
        }
        if (abs(vse[retangulosVaca[l]].y-vse[marcador].y)<altura){
          supsumx += vsd[retangulosVaca[l]].x;
          supsumxsq += pow(vsd[retangulosVaca[l]].x, 2);
          supsumy += vsd[retangulosVaca[l]].y;
          supsumxy += vsd[retangulosVaca[l]].x * vsd[retangulosVaca[l]].y;
          supcontador++;
        }
        contador++;
    }
    denom = supcontador * supsumxsq - pow(supsumx, 2);
    supa1 = (supcontador * supsumxy - supsumx*supsumy)/denom;
    supa0 = (supsumy - supa1*supsumx)/supcontador;

    float porcentagem = 0.8;

    if (tipo==5){
      //*
      pt1linha1.x = pt1linha1.x*(1-porcentagem)+porcentagem*vsd[marcador].x;
      pt1linha1.y = pt1linha1.y*(1-porcentagem)+porcentagem*(vsd[marcador].x*supa1+supa0);
      pt2linha1.x = pt2linha1.x*(1-porcentagem)+porcentagem*vid[marcador].x;
      pt2linha1.y = pt2linha1.y*(1-porcentagem)+porcentagem*(vid[marcador].y);//*a1+a0);

      //Verificação Not a Number
      if (pt1linha1.y!=pt1linha1.y) pt1linha1.y = vsd[marcador].y;
    }
    else{
      //*
      pt1linha1.x = pt1linha1.x*(1-porcentagem)+porcentagem*vse[marcador].x;
      pt1linha1.y = pt1linha1.y*(1-porcentagem)+porcentagem*(vse[marcador].x*supa1+supa0);
      pt2linha1.x = pt2linha1.x*(1-porcentagem)+porcentagem*vie[marcador].x;
      pt2linha1.y = pt2linha1.y*(1-porcentagem)+porcentagem*(vie[marcador].y);//x*a1+a0);
      //*/

      //Verificação Not a Number
      if (pt1linha1.y!=pt1linha1.y) pt1linha1.y = vse[marcador].x*supa1+supa0;
    }

    line(tempBlackWhite, pt1linha1, pt2linha1, Scalar (0, 20, 255), 2, 8 );
    ///*
    pt1linha2.x = pt1linha2.x*(1-porcentagem)+porcentagem*vsd[minmarcador].x;
    pt1linha2.y = pt1linha2.y*(1-porcentagem)+porcentagem*(vsd[minmarcador].x*supa1+supa0);
    pt2linha2.x = pt2linha2.x*(1-porcentagem)+porcentagem*vid[minmarcador].x;
    pt2linha2.y = pt2linha2.y*(1-porcentagem)+porcentagem*(vid[minmarcador].y);
    //*/

    //Verificação Not a Number
    if (pt1linha2.x!=pt1linha2.x) pt1linha2.x = vsd[minmarcador].x;
    if (pt1linha2.y!=pt1linha2.y) pt1linha2.y = vsd[minmarcador].y;//x*supa1+supa0;
    if (pt2linha2.x!=pt2linha2.x) pt2linha2.x = vid[minmarcador].x;
    if (pt2linha2.y!=pt2linha2.y) pt2linha2.y = vid[minmarcador].y;

    line(tempBlackWhite, pt1linha2, pt2linha2, Scalar (0, 20, 255), 2, 8 );

    position(pt2linha1.y-pt1linha1.y, pt2linha2.y-pt1linha2.y, pt1linha1.x,pt1linha2.x);
    matfinal = tempBlackWhite.clone();

  }
  
  cout << "Error = ";
  cout << err;
  cout << '\n';
  cout << "Max Error = ";
  cout << max_error;
  cout << '\n';
  
  if(err < max_error)
  {
    cout << "Got it!\n";
    imshow("Output", tempBlackWhite);
  } else {
    cout << "Oops, didn't get it right\n";
    cvtColor(tempBlackWhite, tempBlackWhite, CV_RGB2GRAY);
    imshow("Output", tempBlackWhite);
  }
  
}

void position (float line1size, float line2size, float px1, float px2){
  float xcenter = src.cols/2;
  z1 = (FOCAL_DIST*10.0)/line1size;
  z2 = (FOCAL_DIST*10.0)/line2size;
  float X1_CENTER_ERROR = X_ERROR_MI*z1 + X_ERROR_CONST;
  float X2_CENTER_ERROR = X_ERROR_MI*z2 + X_ERROR_CONST;
  x1 = (z1*(px1-xcenter)/FOCAL_DIST)-X1_CENTER_ERROR;
  x2 = (z2*(px2-xcenter)/FOCAL_DIST)-X2_CENTER_ERROR;
  dist = pow(pow(z1-z2, 2) + pow(x1-x2, 2) , 0.5)+3.0;
  
  err = abs(dist-36.0)/36.0;
  if(err < max_error)
  { 
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
}

void savePoints (){
  FILE *fp;
  fp = fopen("pontos.txt", "w");

  fprintf (fp, "vecz1\n");
  for (std::vector<float>::iterator it = vecz1.begin() ; it != vecz1.end(); ++it){fprintf (fp, "%f\n", *it); }
  fprintf (fp, "vecx1\n");
  for (std::vector<float>::iterator it =  vecx1.begin() ; it != vecx1.end(); ++it){ fprintf (fp, "%f\n", *it); }
  fprintf (fp, "vecang1\n");
  for (std::vector<float>::iterator it = vecang1.begin() ; it != vecang1.end(); ++it){    fprintf (fp, "%f\n", *it);  }
  fprintf (fp, "vecz2\n");
  for (std::vector<float>::iterator it =  vecz2.begin() ; it != vecz2.end(); ++it){    fprintf (fp, "%f\n", *it);  }
  fprintf (fp, "vecx2\n");
  for (std::vector<float>::iterator it = vecx2.begin() ; it != vecx2.end(); ++it){    fprintf (fp, "%f\n", *it);  }
  fprintf (fp, "vecang2\n");
  for (std::vector<float>::iterator it = vecang2.begin() ; it != vecang2.end(); ++it){    fprintf (fp, "%f\n", *it);  }
  fprintf (fp, "vecdist\n");
  for (std::vector<float>::iterator it = vecdist.begin() ; it != vecdist.end(); ++it){    fprintf (fp, "%f\n", *it);  }
  fclose (fp);
}
