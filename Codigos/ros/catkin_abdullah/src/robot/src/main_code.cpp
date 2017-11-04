	#include <math.h>
	#include <vector>
	#include <iostream>
	#include <fstream>
	using namespace std;

	#define PI 3.14159265f

/*---------------------------------------definicoes ROS-------------------------------------------*/
	#include "ros/ros.h"
	#include "ros/time.h"
	#include "arduino_msgs/StampedInt32.h"
	#include "arduino_msgs/StampedInt64.h"
	#include "arduino_msgs/StampedFloat32.h"
	#include "arduino_msgs/StampedFloat64.h"	

	ros::Publisher pubM_int64;
	ros::Publisher pubM_float64;
	ros::Publisher pubN_int32;
	ros::Publisher pubN_float32;

	ros::Publisher pub_visaoGarra;
	ros::Subscriber sub_visaoGarra;

	ros::Subscriber subM_int64;
	ros::Subscriber subM_float64;
	ros::Subscriber subN_int32;
	ros::Subscriber subN_float32;

	ros::Publisher pubVis_int32;
	ros::Subscriber subVis_float64;

/*------------------------------------------------------------------------------------------------*/
/*-----------------------------------declaracoes das funcoes--------------------------------------*/
	bool ehSonar(int id);
	bool ehToque(int id);
	void messageMInt64Cb( const arduino_msgs::StampedInt64& aM_int64_msg);
	void messageMFloat64Cb( const arduino_msgs::StampedFloat64& aM_float64_msg);
	void messageNInt32Cb( const arduino_msgs::StampedInt32& aN_int32_msg);
	void messageNFloat32Cb( const arduino_msgs::StampedFloat32& aN_float32_msg);
	void messageVisFloat64Cb( const arduino_msgs::StampedFloat64& vis_float64_msg);
	void visaoGarraCB(const arduino_msgs::StampedFloat32& visaoGarra_msg);
	void initROS();
	void SendFloatMega(int id, double data);
	void SendIntMega(int id, long long int data);	
	void SendFloatUno(int id, float data);
	void SendIntUno(int id, long int data);
	void SendVel(float DIR, float ESQ);
	void GiraEmGraus(float angulo);
	float DeGrausParaRadianos(float angulo);
	float DeRadianosParaDegraus(float angulo);
	vector<float> AnaliseLugarDaVaca(float x1, float y1, float x2, float y2, float theta);
	void TrajetoriaSuaveAteAVaca(float x1, float y1, float x2, float y2, float theta);
	void andaRetoDistVel(float dist, float vel);
	void algoritmo();
	void atualizaLocalizacao(int id, float value);
	void inicializarVariaveis();
	vector<int> posicaoPraMatriz(int x, int y);
	vector<float> matrizPraPosicao(int i, int j);
	void preencheMatriz();
	double rotacoesPorSegundoParaLinear(float dado);
	bool ehGarra(int id);
	void SendIntVisaoGarra(int id, long int data);
	void andaComIntegracao(float distancia, float vel);
	void buscaMelhorCaminho(int NoAtual, int iO, int jO);
	class NoDoMapa;
	class Vertice;
	int deIJparaCasaNoVetor(int i, int j);
	vector<int> deCasaNoVetorParaIJ(int casaNoVetor);
	void printaGrafo();
	bool criaGrafoComPesos();
	void buscaMelhorCaminho(int NoAtual, int iO, int jO);

/*------------------------------------------------------------------------------------------------*/

/*-----------------------------------definicoes mapeamento----------------------------------------*/
	#define CASA_INICIAL_I 0
	#define CASA_INICIAL_J 0
	
	#define CASA_I_BALDE  
	#define CASA_J_BALDE 

	#define CASA_I_VACA1 
	#define CASA_J_VACA1 

	#define CASA_I_VACA2 
	#define CASA_J_VACA2 

	//////////////define na hora que a posicao eh escolhida 
	
	#define CASA_I_MESA_PEGAR 4
	#define CASA_J_MESA_PEGAR 6

	#define CASA_I_MESA_DEVOLVER 5
	#define CASA_J_MESA_DEVOLVER 1
	
	vector<float> ladosQuadrado(2);	
	
	#define I 0
	#define J 1

	#define COLUNAS_MAPA 6
	#define LINHAS_MAPA 8
	//vector<vector<int> > mapa(LINHAS_MAPA, vector<int>(COLUNAS_MAPA));
	class Vertice{
		public:	
			double peso;
	        NoDoMapa* noAdjacente;
	};

	class NoDoMapa{
		public:
			vector<float> centro;
	        int NumeroDoNo;
	        int informacaoDoQuadrado;
	        bool Visitado;
			vector<Vertice> adjacentes;
			void adicionaAdjacente(int casaNo, int peso);
		NoDoMapa(){
			centro = vector<float>(2);
		}
	};


	vector<vector<NoDoMapa> > GrafoMapa(LINHAS_MAPA, vector<NoDoMapa>(COLUNAS_MAPA));

/*------------------------------------------------------------------------------------------------*/

/*-------------------------------------definicoes locomocao---------------------------------------*/

	#define VELOCIDADE_MAXIMA 1.3f
	#define VELOCIDADE_NORMAL 1.0f
	#define VELOCIDADE_BAIXA 0.5f
	#define VELOCIDADE_BAIXISSIMA 0.2f
	
	#define CORRECAO_GIRO 9

	#define GIRA 300
	#define VEL_REF_DIR 301
	#define VEL_REF_ESQ 302
	#define TRAVAR 303
	#define VELOCIDADE_CRU 304
	#define PARAMETRO_DERRAPAGEM_ROTACOES_METROS 8	
	#define DIAMETRO_MEDIO 9.5125f

	vector<float> posicao(3);

	double distancia_integracao;
	
	#define X 0
	#define Y 1
	#define ANGULO 2
	
	
	#define ACABOU_GIRO 1
	#define ACABOU_ATUAL 2
	#define DISTANCIA_MIN_AJUSTE_VACA 30.0f
	
	#define ANDAR 1
	#define GIRAR 2
/*------------------------------------------------------------------------------------------------*/
/*---------------------------------definicoes dos sensores US-------------------------------------*/

	#define US1 0
	#define US2 1
	#define US3 2
	#define US4 3
	#define US5 4
	#define US6 5
	#define US7 6
	#define US8 7
	#define US_MAX_DIST 200
	#define NUM_IDEN_US 100
	#define QUANTIDADE_SENSOR_US 8
	#define TAMANHO_MEDIANA 5
	#define VALOR_MEDIANA 2
	//sinal chega do arduino com o id(USn) = (n-1) + NUM_IDEN_US... exemplo: id do US5 = (5-1) + 100 = 104

	#define alfa_US 0.8f

	class Us{
  		public:
	  		float valor;
	  		vector<float> valores;
	  		long long int vezes_lido;
		Us(){
			valores = vector<float>(TAMANHO_MEDIANA);
		}
	};

	vector<Us> ultrassom(QUANTIDADE_SENSOR_US);
/*------------------------------------------------------------------------------------------------*/

/*-----------------------------definicoes dos sensores de TOQUE-----------------------------------*/
	#define TOQUE1 0
	#define TOQUE2 1
	#define TOQUE3 2
	#define TOQUE4 3
	#define TOQUE5 4
	#define TOQUE6 5
	#define TOQUE7 6

	#define TOQUE8 7
	#define TOQUE9 8

	#define NUM_IDEN_TOQUE 200
	#define QUANTIDADE_SENSOR_TOQUE 9

	vector<bool> toque(QUANTIDADE_SENSOR_TOQUE);//(QUANTIDADE_SENSOR_TOQUE, false);
/*------------------------------------------------------------------------------------------------*/
/*--------------------------------------Garra-----------------------------------------------------*/
	#define QUANTIDADE_MOTORES_GARRA 5
	
	#define MOTOR_PASSO_X 0 
	#define MOTOR_PASSO_Y 1
	#define MOTOR_SERVO_COTOVELO 2
	#define MOTOR_SERVO_PULSO 3 
	#define MOTOR_SERVO_ATUADOR 4

	#define ACABOU_GARRA 5
	
	#define ANDA_PRA_FRENTE_GARRA 666 
	
	#define NUM_IDEN_GARRA 600

	#define VELOCIDADE_COPO 0.24f

	#define PEGA_COPO 10
	#define DESCARREGA_COPO 20 
	#define DEVOLVE_COPO 30
/*------------------------------------------------------------------------------------------------*/

/*-----------------------------definicoes da VISAO-----------------------------------*/
	#define NUM_IDEN_VISION 500
	double cow_pos_x1, cow_pos_x2, cow_pos_z1, cow_pos_z2, cow_pos_err;

/*------------------------------------------------------------------------------------------------*/

class Ocupacao{
	public:
		bool giro;
		bool garraGeral;
};

Ocupacao ocupado;
vector<int> SequenciaDeNosParaSeguir;



int deIJparaCasaNoVetor(int i, int j)
{
	return i*COLUNAS_MAPA+j;
}

vector<int> deCasaNoVetorParaIJ(int casaNoVetor)
{
	vector<int> casaMatrix(2);
	casaMatrix[0] = floor(casaNoVetor/COLUNAS_MAPA);
	casaMatrix[1] = casaNoVetor - casaMatrix[0]*COLUNAS_MAPA;
	return casaMatrix;
}

void NoDoMapa::adicionaAdjacente(int casaNo, int peso)
{
	Vertice temp;
    temp.peso = peso;
    vector<int> casaMatrix(2);
    casaMatrix = deCasaNoVetorParaIJ(casaNo);
    int i = casaMatrix[0];
    int j = casaMatrix[1];
    temp.noAdjacente = & (GrafoMapa[i][j]);
    adjacentes.push_back(temp);
}
void printaGrafo()//funcao para imprimir o grafo
{     
        for (int i = 0; i<LINHAS_MAPA; ++i)
        {
	       	for (int j = 0; j < COLUNAS_MAPA; ++j)
	       	{
	       		
	            cout << "no: "<<GrafoMapa[i][j].NumeroDoNo<< endl <<" x postition "<<GrafoMapa[i][j].centro[X]<<"  y postition "<<GrafoMapa[i][j].centro[Y]<<endl;///numero do no = i+1, seria numerado pelas casas no vetor
	            cout<< endl << "\tadjacents ";
	            for(int k=0 ; k < GrafoMapa[i][j].adjacentes.size() ; k++)
	                cout << GrafoMapa[i][j].adjacentes[k].noAdjacente->NumeroDoNo << " ";
	            cout<< endl << "\tPesos ";
	            for(int k=0 ; k < GrafoMapa[i][j].adjacentes.size() ; k++)
	                cout << GrafoMapa[i][j].adjacentes[k].peso << " ";
	            cout<< endl;
	       	}
        }
}

bool criaGrafoComPesos()
{
	int linhas=0,colunas=0,verts,aux_vert=0;
	int peso;
	ifstream GrafoTxt ("/home/abdullah/catkin_abdullah/src/robot/src/grafoMapa.txt"); // abrir arquivo (o programa tem que ser executado na mesma pasta q o arquivo que tem os dados do grafo esta).
	GrafoTxt >> linhas;
	GrafoTxt >> colunas;
	if (GrafoTxt.is_open())
	{  
	    //cout<<"GrafoMapa file is opened !\n";
	    for (int i = 0; (i<linhas) ; ++i)
	    {              
	        for(int j = 0 ; j<colunas; ++j){
	            NoDoMapa temp;
	            temp.centro = matrizPraPosicao(i,j); // calculo para obter as coordenadas x e y do centro do apartir das fronteiras pegas do arquivo texto
	            temp.NumeroDoNo = deIJparaCasaNoVetor(i, j);
	            temp.Visitado = 0;
	            GrafoTxt >> verts;
	            for (int k = 0; (k<verts); ++k)
	            {
	                GrafoTxt >> aux_vert;
	                GrafoTxt >> peso;
	                temp.adicionaAdjacente(aux_vert,peso);
	            }
	            GrafoMapa[i][j]=temp;
	        }   
	    }
	    return true;       
	}else{
	   // cout << "************************************************************************\n"; 
	  //  cout << "GrafoMapa file isn't opened!\nAre you sure that you are in the right directory to execute the program?\nMake sure to be in /catkin_ws/src/fcr2017/src\n";
	  //  cout << "************************************************************************\n";
	    return false;
	}
}

void buscaMelhorCaminho(int NoAtual, int iO, int jO)
{
	double OutrosCaminhos,Min; /// caminhos alternativos ao no atual
	int k;

	vector<NoDoMapa*> NaoVistado; //vetor de ponteiros nao visitados ainda
	NoDoMapa* u = NULL;
	NoDoMapa* v = NULL; //dois ponteiros pra nos que facilitam a escrita do codigo

	vector<NoDoMapa*> SequenciaDeNos(COLUNAS_MAPA*LINHAS_MAPA,NULL); /// aponta nos nos conectados com um dado no, no caminho
	vector<double> Peso(COLUNAS_MAPA*LINHAS_MAPA,999999999); // guarda as distancias ate o no inicial, comecado com distancia mto grande

	for (int i = 0; i < LINHAS_MAPA; ++i)
	{
		for (int j = 0; j < COLUNAS_MAPA; ++j)
		{
			NaoVistado.push_back(&GrafoMapa[i][j]);   
		}

	} /// passando os enderecos dos nos do grafo pro vetor de ponteiros n visitados ()

	Peso[NoAtual] = 0;
	    ///distancia do no pro proprio no e zero
	while(NaoVistado.size()>0 && u != &GrafoMapa[iO][jO])
	{
	    v = NaoVistado[0];
	    Min = Peso[v->NumeroDoNo];
	    u = v;
	    k=0;
	    for (int i = 0; i < NaoVistado.size(); i++)
	    {
	        v = NaoVistado[i];
	        if (Peso[v->NumeroDoNo]<Min)
	        {
	            Min = Peso[v->NumeroDoNo];
	            u = v;
	            k= i;
	        }
	    }
	    NaoVistado.erase(NaoVistado.begin()+k);

	    for (int i = 0; i < u->adjacentes.size(); ++i)
	    {
	        v = u->adjacentes[i].noAdjacente;
	        OutrosCaminhos = Peso[u->NumeroDoNo] + u->adjacentes[i].peso;
	        if (OutrosCaminhos < Peso[v->NumeroDoNo])
	        {
	            Peso[v->NumeroDoNo] = OutrosCaminhos;
	            SequenciaDeNos[v->NumeroDoNo] = u;
	        }

	    }
	}

	vector<NoDoMapa> UltimaSequencia;
	while(SequenciaDeNos[u->NumeroDoNo] != NULL)
	{
	    UltimaSequencia.insert(UltimaSequencia.begin(),*u);
	    u = SequenciaDeNos[u->NumeroDoNo]; 
	}

	UltimaSequencia.insert(UltimaSequencia.begin(),*u);
	for (int i = 1; i < UltimaSequencia.size(); ++i)
	    SequenciaDeNosParaSeguir.push_back(UltimaSequencia[i].NumeroDoNo);
}

template<typename ItemType>
unsigned Partition(ItemType* array, unsigned f, unsigned l, ItemType pivot)
{
    unsigned i = f-1, j = l+1;
    while(true)
    {
        while(pivot < array[--j]);
        while(array[++i] < pivot);
        if(i<j)
        {
            ItemType tmp = array[i];
            array[i] = array[j];
            array[j] = tmp;
        }
        else
            return j;
    }
}

template<typename ItemType>
void QuickSortImpl(ItemType* array, unsigned f, unsigned l)
{
    while(f < l)
    {
        unsigned m = Partition(array, f, l, array[f]);
        QuickSortImpl(array, f, m);
        f = m+1;
    }
}

template<typename ItemType>
void QuickSort(ItemType* array, unsigned size)
{
    QuickSortImpl(array, 0, size-1);
}

bool ehSonar(int id)
{
	id = id - NUM_IDEN_US;
	if (id < QUANTIDADE_SENSOR_US && id >= 0)	 return true;
	else	return false;
}

bool ehToque(int id)
{
	id = id - NUM_IDEN_TOQUE;
	if (id < QUANTIDADE_SENSOR_TOQUE && id >= 0)	 return true;
	else	return false;
}

bool ehGarra(int id)
{
	id = id - NUM_IDEN_GARRA;
	 // o +1 eh para considerar o caso quando a garra acaba o movimento
	if (id < QUANTIDADE_MOTORES_GARRA+1 && id >= 0)	 return true;
	else	return false;
}


/*
bool verificacaoDado(int tipo, int id)
{
	switch(tipo)
	{
		case :
	}
}
*/
void Delay(double time)
{
    double t1=0, t0=0;
    t0 = ros::Time::now().toSec();
    while((t1-t0)<time && ros::ok()){
        t1 = ros::Time::now().toSec();
        ros::spinOnce();
    }
}
/*----------------------------------------callbacks------------------------------------------------*/
	void messageMInt64Cb( const arduino_msgs::StampedInt64& aM_int64_msg)
	{
		if(ehSonar(aM_int64_msg.id))
		{
			int usPos = aM_int64_msg.id - NUM_IDEN_US; 
			int valor ;
			valor = (aM_int64_msg.data <= 0.1) ? US_MAX_DIST : aM_int64_msg.data;	
			//sonar[aN_int32_msg.id - NUM_IDEN_US] = valor * alfa_US + sonar[aN_int32_msg.id - NUM_IDEN_US] * (1 - alfa_US);
			float valorf = (float)valor ; 
			long long int iterator = ultrassom[usPos].vezes_lido ;
		    ultrassom[usPos].valores[iterator % TAMANHO_MEDIANA] = valorf;
		    float valores[TAMANHO_MEDIANA];
		    for(unsigned i = 0; i < TAMANHO_MEDIANA; ++i)
		      	valores[i] = ultrassom[usPos].valores[i];
		    QuickSort(valores,TAMANHO_MEDIANA);
		    //ultrassom[usPos].value = values[MEDIAN_VALUE];
		    valorf = valores[VALOR_MEDIANA];
		    ultrassom[usPos].valor = valorf * alfa_US + ultrassom[usPos].valor * ( 1- alfa_US);
		    ultrassom[usPos].vezes_lido++;
		}else if (ehToque(aM_int64_msg.id)) 
			toque[aM_int64_msg.id - NUM_IDEN_TOQUE] = aM_int64_msg.data;
		else if (ehGarra(aM_int64_msg.id)){
			SendIntVisaoGarra(aM_int64_msg.id,aM_int64_msg.data);
		}
	}


	void messageMFloat64Cb( const arduino_msgs::StampedFloat64& aM_float64_msg)
	{
		
	}

	void messageNInt32Cb( const arduino_msgs::StampedInt32& aN_int32_msg)
	{
		if (aN_int32_msg.id == ACABOU_GIRO)		ocupado.giro = false;
	}

	void messageNFloat32Cb( const arduino_msgs::StampedFloat32& aN_float32_msg)
	{
		if (aN_float32_msg.id == VELOCIDADE_CRU)		distancia_integracao += 100*(rotacoesPorSegundoParaLinear(aN_float32_msg.data)* 0.020004);
		//o 100 é para mudar de metros apra centimetros (medida que a gente usa no codigo)
	}

	void visaoGarraCB(const arduino_msgs::StampedFloat32& visaoGarra_msg)
	{
		if (ehGarra(visaoGarra_msg.id)){
			ocupado.garraGeral = (visaoGarra_msg.id == ACABOU_GARRA + NUM_IDEN_GARRA) ? false : true;	
			SendIntMega(visaoGarra_msg.id, visaoGarra_msg.data);
		}
		else if(visaoGarra_msg.id == ANDA_PRA_FRENTE_GARRA){
			andaComIntegracao(visaoGarra_msg.data, VELOCIDADE_COPO);
		}
	}

	void messageVisFloat64Cb( const arduino_msgs::StampedFloat64& vis_float64_msg)
	{
		if(vis_float64_msg.id == NUM_IDEN_VISION + 1)
		{
			//cout << "Got x1: ";
			cow_pos_x1 = vis_float64_msg.data;
			//cout << vis_float64_msg.data;
			//cout << "\n";
		} else if(vis_float64_msg.id == NUM_IDEN_VISION + 2) {
			//cout << "Got z1: ";
			cow_pos_z1 = vis_float64_msg.data;
			//cout << vis_float64_msg.data;
			//cout << "\n";
		} else if(vis_float64_msg.id == NUM_IDEN_VISION + 3) {
			//cout << "Got x2: ";
			cow_pos_x2 = vis_float64_msg.data;
			//cout << vis_float64_msg.data;
			//cout << "\n";
		} else if(vis_float64_msg.id == NUM_IDEN_VISION + 4) {
			//cout << "Got z2: ";
			cow_pos_z2 = vis_float64_msg.data;
			//cout << vis_float64_msg.data;
			//cout << "\n";
		} else if(vis_float64_msg.id == NUM_IDEN_VISION + 5) {
			//cout << "Got error value: ";
			cow_pos_err = vis_float64_msg.data;
			//cout << vis_float64_msg.data;
			//cout << "\n";
		} else {
			cout << "Got something weird\n";
		}
	}
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------comunicacao ROS---------------------------------------------*/
	
	void initROS(ros::NodeHandle nh)
	{
		pubM_int64 = nh.advertise<arduino_msgs::StampedInt64>("raspberryM_int64", 1000);
		pubM_float64 = nh.advertise<arduino_msgs::StampedFloat64>("raspberryM_float64", 1000);
		subM_int64 = nh.subscribe("arduinoM_int64", 1000, messageMInt64Cb);
		subM_float64 = nh.subscribe("arduinoM_float64", 1000, messageMFloat64Cb);
		
		pubN_int32 = nh.advertise<arduino_msgs::StampedInt32>("raspberryN_int32", 1000);
		pubN_float32 = nh.advertise<arduino_msgs::StampedFloat32>("raspberryN_float32", 1000);
		subN_int32 = nh.subscribe("arduinoN_int32", 1000, messageNInt32Cb);
		subN_float32 = nh.subscribe("arduinoN_float32", 1000, messageNFloat32Cb);
		
		pub_visaoGarra = nh.advertise<arduino_msgs::StampedInt32>("estrategiaGarra", 1000); 
		sub_visaoGarra = nh.subscribe("visaoGarra", 1000, visaoGarraCB);

		pubVis_int32 = nh.advertise<arduino_msgs::StampedInt32>("Vision_int32", 1000); 
		subVis_float64 = nh.subscribe("Vision_float64", 1000, messageVisFloat64Cb);

	}

	void SendFloatMega(int id, double data)
	{   
		arduino_msgs::StampedFloat64 float64_msg;
		float64_msg.id = id;
		float64_msg.data = data;
		pubM_float64.publish(float64_msg);
	}


	void SendIntMega(int id, long long int data)
	{    
		arduino_msgs::StampedInt64 int64_msg;
		int64_msg.id = id;
		int64_msg.data = data;
		pubM_int64.publish(int64_msg);
	}

	void SendIntVisaoGarra(int id, long int data)
	{    
		arduino_msgs::StampedInt32 int32_msg;
		int32_msg.id = id;
		int32_msg.data = data;
		pub_visaoGarra.publish(int32_msg);
	}


	void SendFloatUno(int id, float data)
	{    
		arduino_msgs::StampedFloat32 float32_msg;
		float32_msg.id = id;
		float32_msg.data = data;
		pubN_float32.publish(float32_msg);
	}

	void SendIntUno(int id, long int data)
	{    
		arduino_msgs::StampedInt32 int32_msg;
		int32_msg.id = id;
		int32_msg.data = data;
		pubN_int32.publish(int32_msg);
	}

	void SendIntVision(int id, long int data)
	{
		arduino_msgs::StampedInt32 int32_msg;
		int32_msg.id = id;
		int32_msg.data = data;
		pubVis_int32.publish(int32_msg);
	}

	void SendVel(float DIR, float ESQ)
	{
		SendFloatUno(VEL_REF_DIR, DIR);
		SendFloatUno(VEL_REF_ESQ, ESQ);
	}
/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------conversoes--------------------------------------------------*/
	float DeGrausParaRadianos(float angulo)
	{
		return (angulo*PI)/180;
	}

	float DeRadianosParaDegraus(float angulo)
	{
		return (angulo*180)/PI;
	}

	double rotacoesPorSegundoParaLinear(float dado)
	{
		return (dado*(DIAMETRO_MEDIO)/(PARAMETRO_DERRAPAGEM_ROTACOES_METROS*PI));
	}
/*-------------------------------------------------------------------------------------------------*/
/*----------------------------------------andar&Girar----------------------------------------------*/
	void GiraEmGraus(float angulo)
	{
		ocupado.giro = true;
		SendFloatUno(GIRA, angulo - (float(CORRECAO_GIRO)));
		while(ocupado.giro && ros::ok()){		
			ros::spinOnce();
		}
		atualizaLocalizacao(GIRAR, angulo);
	}

	void andaRetoDistVel(float dist, float vel)
	{
		double tempo=0;
		tempo = dist*2.682871209/100*abs(vel); /// de rotacoes por segundo para metros por segundo
		dist = (vel > 0) ? dist : -dist;
		SendVel(vel, vel);
		SendVel(vel, vel);
		Delay(tempo);
		SendVel(0.0,0.0);
		SendFloatUno(TRAVAR,0);
		atualizaLocalizacao(ANDAR, dist);
	}

	void andaComIntegracao(float distancia, float vel)
	{
		distancia_integracao = 0.0;
		SendVel(vel,vel);
		SendVel(vel,vel);
		while(distancia_integracao < distancia && ros::ok())
		{
			ros::spinOnce();		
		}
		distancia = (vel > 0) ? distancia_integracao : -distancia_integracao;
		SendVel(0,0);
		SendFloatUno(TRAVAR,0);
		atualizaLocalizacao(ANDAR, distancia);
	}
/*-------------------------------------------------------------------------------------------------*/

	void garraEstado(int estado)
	{
		ocupado.garraGeral = true;
		switch(estado)
		{
			case PEGA_COPO:
				SendIntVisaoGarra(NUM_IDEN_GARRA+PEGA_COPO, estado/10);
				break;
			case DESCARREGA_COPO:
				SendIntVisaoGarra(NUM_IDEN_GARRA+DESCARREGA_COPO, estado/10);
				break;
			case DEVOLVE_COPO:
				SendIntVisaoGarra(NUM_IDEN_GARRA+DEVOLVE_COPO, estado/10);
				break;
		}
		while(ocupado.garraGeral && ros::ok())
		{
			ros::spinOnce();	
		}
	}
/*-----------------EM CENTIMETROS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!-----------------------*/
/*----------------------------------------mapeamento-----------------------------------------------*/
	vector<int> posicaoPraMatriz(int x, int y)
	{
		vector<int> casasMat(2);
		float aux;
		aux = x/ladosQuadrado[X];
		casasMat[J] = round(aux);
		aux = y/ladosQuadrado[Y];
		casasMat[I] = round(aux);
		return casasMat;
	}

	vector<float> AnaliseLugarDaVaca(float x1, float y1, float x2, float y2, float theta)
	{
		float  distancia_direta, centro_da_entrada[2];
		vector<float> ponto_do_viro(2);
		centro_da_entrada[X] = ((x2-x1)/2);
		centro_da_entrada[Y] = ((y2-y1)/2);
		distancia_direta = sqrt(pow(centro_da_entrada[X],2) + pow(centro_da_entrada[Y],2));
		if (distancia_direta<DISTANCIA_MIN_AJUSTE_VACA)
			///se ajeita (dar re e ganhar espaco para chegar melhor na vaca) e faz de novo o processo
		ponto_do_viro[X] = centro_da_entrada[X] - (DISTANCIA_MIN_AJUSTE_VACA*cos(DeGrausParaRadianos(90-theta)));
		ponto_do_viro[Y] = centro_da_entrada[Y] - (DISTANCIA_MIN_AJUSTE_VACA*sin(DeGrausParaRadianos(90-theta)));
		return ponto_do_viro;
	}
	//y1= 87 x1=10 y2=56 x2 = 31 theta= 55
	void TrajetoriaSuaveAteAVaca(float x1, float y1, float x2, float y2, float theta)
	{
		float alfa, distancia_ate_ponto_do_giro;
		vector<float> v = AnaliseLugarDaVaca(x1, y1, x2, y2, theta);
		alfa = DeRadianosParaDegraus(atan2(v[X],v[Y]));
		GiraEmGraus(alfa);
		distancia_ate_ponto_do_giro = sqrt(pow(v[X],2) + pow(v[Y],2));
		andaRetoDistVel(distancia_ate_ponto_do_giro, VELOCIDADE_NORMAL);
		GiraEmGraus(90-(alfa+theta));
		andaRetoDistVel(DISTANCIA_MIN_AJUSTE_VACA, VELOCIDADE_BAIXA);
	}

	void ChegaNaVaca()
	{
		float dist;
		float xmed_vaca = (cow_pos_x2 + cow_pos_x1)/2;
		float zmed_vaca = (cow_pos_z2 + cow_pos_z1)/2;
		float ang_vaca = atan2(cow_pos_z2 - cow_pos_z1, cow_pos_x2 - cow_pos_x1) - 90; // ângulo da vaca em relação ao eixo Z
		float ang_robovaca = atan2(zmed_vaca, xmed_vaca) - 90; // ângulo do centroide da vaca em relação ao eixo Z
		float dist_centro_vaca = sqrt(pow(xmed_vaca, 2) + pow(zmed_vaca, 2));
		
		//O trecho seguinte movimenta o robô de modo a se orientar de maneira frontal à vaca,
		//na tentativa de melhorar a imagem da vaca obtida pela visão.
		if(ang_robovaca > 0)	// se a vaca tá mais à esquerda do robô
		{
			if(ang_vaca < 0)		// se o lado da vaca que é bom de ordenhar tá virado pro robô
			{
				GiraEmGraus(ang_vaca); // gira pra ficar de lado pra a vaca
				dist = dist_centro_vaca*cos(ang_vaca - ang_robovaca);
				andaRetoDistVel(dist, VELOCIDADE_NORMAL);
				GiraEmGraus(-90);
			} else {
				GiraEmGraus(90);
				dist = abs(xmed_vaca) + 10;
				andaRetoDistVel(dist, VELOCIDADE_NORMAL);
				GiraEmGraus(-90);
				//atualiza os valores de cow_pos
				xmed_vaca = (cow_pos_x2 + cow_pos_x1)/2;
				zmed_vaca = (cow_pos_z2 + cow_pos_z1)/2;
				ang_vaca = atan2(cow_pos_z2 - cow_pos_z1, cow_pos_x2 - cow_pos_x1) - 90; // ângulo da vaca em relação ao eixo Z
				ang_robovaca = atan2(zmed_vaca, xmed_vaca) - 90; // ângulo do centroide da vaca em relação ao eixo Z
				dist_centro_vaca = sqrt(pow(xmed_vaca, 2) + pow(zmed_vaca, 2));
				GiraEmGraus(ang_vaca); // gira pra ficar de lado pra a vaca
				dist = dist_centro_vaca*cos(ang_vaca - ang_robovaca);
				andaRetoDistVel(dist, VELOCIDADE_NORMAL);
				GiraEmGraus(-90);
			}
		} else {				// se a vaca tá mais à direita do robô
			if(ang_vaca < 0)		// se o lado da vaca que é bom de ordenhar tá virado pro robô
			{
				GiraEmGraus(ang_vaca); // gira pra ficar de lado pra a vaca
				dist = dist_centro_vaca*cos(ang_vaca - ang_robovaca);
				andaRetoDistVel(dist, VELOCIDADE_NORMAL);
				GiraEmGraus(-90);
			} else {
				GiraEmGraus(-90);
				dist = abs(xmed_vaca) + 10;
				andaRetoDistVel(dist, VELOCIDADE_NORMAL);
				GiraEmGraus(90);
				//atualiza os valores de cow_pos
				xmed_vaca = (cow_pos_x2 + cow_pos_x1)/2;
				zmed_vaca = (cow_pos_z2 + cow_pos_z1)/2;
				ang_vaca = atan2(cow_pos_z2 - cow_pos_z1, cow_pos_x2 - cow_pos_x1) - 90; // ângulo da vaca em relação ao eixo Z
				ang_robovaca = atan2(zmed_vaca, xmed_vaca) - 90; // ângulo do centroide da vaca em relação ao eixo Z
				dist_centro_vaca = sqrt(pow(xmed_vaca, 2) + pow(zmed_vaca, 2));
				GiraEmGraus(ang_vaca); // gira pra ficar de lado pra a vaca
				dist = dist_centro_vaca*cos(ang_vaca - ang_robovaca);
				andaRetoDistVel(dist, VELOCIDADE_NORMAL);
				GiraEmGraus(-90);
			}
		}
	}

	void atualizaLocalizacao(int id, float value)
	{
		if (id == ANDAR){
			posicao[X] += value * cos(DeGrausParaRadianos(posicao[ANGULO]));
			posicao[Y] += value * sin(DeGrausParaRadianos(posicao[ANGULO])); 
		}else if (id == GIRAR)		posicao[ANGULO] += value;			
	}

	vector<float> matrizPraPosicao(int i, int j)
	{
		vector<float> pos(2);
		pos[X] = ((j)*ladosQuadrado[X])+(ladosQuadrado[X]/2);
		pos[Y] = ((i)*ladosQuadrado[Y])+(ladosQuadrado[Y]/2);
		return pos;
	}
/*-------------------------------------------------------------------------------------------------*/
/*void preencheMatriz()
{
	for (int i = 0; i < LINHAS_MAPA; ++i)
	{
		for (int j = 0; j < COLUNAS_MAPA; ++j)
		{
			for (int k = 0; k < 2; ++k)
			{
				if(k==0)	mapa[i][j][k] = 0;
				else if ((j == 0  || j == COLUNAS_MAPA-1) && k == 1)		mapa[i][j][k] = 10;
				else if (i == 0 || i == LINHAS_MAPA-1)	mapa[i][j][k] = 5;
				else if (((i == COLUNAS_MAPA/2 || i == COLUNAS_MAPA/2) || (i == COLUNAS_MAPA/2+1 || i == COLUNAS_MAPA/2+1)) && k == 1)		mapa[i][j][k] = 7;
			}
		}
	}
	
	for (int k = 0; k < 2; ++k)	
	{
		for (int i = 0; i < LINHAS_MAPA; ++i)
		{
			for (int j = 0; j < COLUNAS_MAPA; ++j)	std::cout << std::setfill('0') << std::setw(2) << mapa[i][j][k] << " ";
			cout<<endl;
		}
		cout<<endl;	
	}
}*/

void inicializarVariaveis()
{
	ocupado.giro = false;
	fill(toque.begin(), toque.end(), true);
	ocupado.garraGeral = false;
	ladosQuadrado[X] = 300/(COLUNAS_MAPA);
	ladosQuadrado[Y] = 400/(LINHAS_MAPA);
	vector<float> aux(2);
	aux = matrizPraPosicao(CASA_INICIAL_I-1,CASA_INICIAL_J-1);
	posicao[X] = aux[X];	posicao[Y] = aux[Y];
	//preencheMatriz();
	distancia_integracao = 0;
	if(criaGrafoComPesos());
	printaGrafo();
}

void printSensor(int id)
{
	if (id == 0)
	{
		cout << endl <<"us "<<endl;
		for (int i = 0; i < QUANTIDADE_SENSOR_US; ++i)
		{
			cout << ultrassom[i].valor <<" ";
		}
	}else
	{
		cout << endl <<"toque "<<endl;
		for (int i = 0; i < QUANTIDADE_SENSOR_TOQUE; ++i)
		{
			cout << toque[i] << " ";
		}
		cout << endl;
	}
}


void algoritmo()
{
	/*Delay(3);
	andaComIntegracao(50.0,0.5);
	GiraEmGraus(90);
	SendFloatMega(100,distancia_integracao);
	*/
	double t0 = ros::Time::now().toSec();
	buscaMelhorCaminho(deIJparaCasaNoVetor(1,0),6,5);
	double t1 = ros::Time::now().toSec();
	for (int i = 0; i < SequenciaDeNosParaSeguir.size(); ++i)
		cout<<SequenciaDeNosParaSeguir[i]<<"\t";
	cout<<endl<<"tempo de busca= " << t1-t0<<endl;
	Delay(30);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "main_node");
	ros::NodeHandle nh;
	initROS(nh);
	inicializarVariaveis();
	
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		algoritmo();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}