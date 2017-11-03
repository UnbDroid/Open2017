//Conexões feitas seguindo o modelo no seguinte site:
//  https://jgamblog.wordpress.com/2016/09/24/tutorial-arduino-drv8825-motor-de-passo/


// Controle de Motor de Passo Bipolar com Modulo driver DRV8825
// https://jgamblog.wordpress.com/2016/09/24/tutorial-arduino-drv8825-motor-de-passo/
// Modulo DRV8825 / Arduino Nano / IDE 1.6.7
// OBS: Motor Unipolar 6 fios configurado como Bipolar
// Gustavo Murta 26/set/2016

// Definições das Portas Digitais do Arduino

int RST = 8;              // Porta digital D08 - reset do DRV8825
int SLP = 9;              // Porta digital D09 - dormir (sleep) DRV8825
int ENA = 7;              // Porta digital D07 - ativa (enable) DRV8825
int M0 = 4;               // Porta digital D04 - M0 do DRV8825
int M1 = 5;               // Porta digital D05 - M1 do DRV8825
int M2 = 6;               // Porta digital D06 - M2 do DRV8825
int DIR = 3;              // Porta digital D03 - direção (direction) do DRV8825
int STP = 2;              // Porta digital D02 - passo(step) do DRV8825

int MeioPeriodo = 500;   // MeioPeriodo no pulso em microsegundos  correcao de +10 ms 1490
float PPS = 200;          // Pulsos por segundo
boolean sentido = true;   // Variavel de sentido
int PPR = 200;            // Número de passos por volta
int Pulsos;               // Pulsos para o driver do motor
int Voltas;               // voltas do motor
float RPM;                // Rotacoes por minuto

void setup()
{
  Serial.begin(9600);
  DDRD = DDRD | B11111100;  // Configura Portas D02 até D07 como saída
  disa_DRV8825();           // Desativa as saidas DRV8825
  FULL();                    // Seleciona modo Passo Completo

  DDRB = 0x0F;              // Configura Portas D08,D09,D10 e D11 como saída
  digitalWrite(SLP, HIGH);  // Desativa modo sleep do DRV8825
  rst_DRV8825();            // Reseta o chip DRV8825
  digitalWrite(ENA, LOW);   // Ativa as saidas DRV8825
}

void rst_DRV8825()
{
  digitalWrite(RST, LOW);     // Realiza o reset do DRV8825
  delay (1);                  // Atraso de 1 milisegundo
  digitalWrite(RST, HIGH);    // Libera o reset do DRV8825
  delay (10);                 // Atraso de 10 milisegundos
}

void disa_DRV8825()
{
  digitalWrite(ENA, HIGH);    // Desativa o chip DRV8825
  delay (10);                 // Atraso de 10 milisegundos
}

void ena_DRV8825()
{
  digitalWrite(ENA, LOW);     // Ativa o chip DRV8825
  delay (10);                 // Atraso de 10 milisegundos
}

void HOR()                                    // Configura o sentido de rotação do Motor
{
  digitalWrite(DIR, HIGH);                    // Configura o sentido HORÁRIO
  Serial.print(" Sentido Horario ");
}

void AHR()                                    // Configura o sentido de rotação do Motor
{
  digitalWrite(DIR, LOW);                     // Configura o sentido ANTI-HORÁRIO
  Serial.print(" Sentido Anti-horario ");
}

void PASSO()                    // Pulso do passo do Motor
{
  digitalWrite(STP, LOW);            // Pulso nível baixo
  delayMicroseconds (MeioPeriodo);   // MeioPeriodo de X microsegundos
  digitalWrite(STP, HIGH);           // Pulso nível alto
  delayMicroseconds (MeioPeriodo);   // MeioPeriodo de X microsegundos
}

void Frequencia()                     // Configura Frequencia dos pulsos
{
  Pulsos = PPR * Voltas;              // Quantidade total de Pulsos  PPR = pulsos por volta
  PPS = 1000000 / (2 * MeioPeriodo);  // Frequencia Pulsos por segundo
  RPM = (PPS * 60) / PPR;             // Calculo do RPM
}

void FULL()
{
  PPR = 200;                // PPR pulsos por volta
  digitalWrite(M0, LOW);    // Configura modo Passo completo (Full step)
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  Serial.println(" Passo Completo ");
}

void HALF()
{
  PPR = 400;                 // PPR pulsos por volta
  digitalWrite(M0, HIGH);    // Configura modo Meio Passo (Half step)
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  Serial.println(" Meio Passo ");
}

void P1_4()
{
  PPR = 800;                // PPR pulsos por volta
  digitalWrite(M0, LOW);    // Configura modo Micro Passo 1/4
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  Serial.println(" Micropasso 1/4 ");
}

void P1_8()
{
  PPR = 1600;                // PPR pulsos por volta
  digitalWrite(M0, HIGH);    // Configura modo Micro Passo 1/8
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  Serial.println(" Micropasso 1/8 ");
}

void P1_16()
{
  PPR = 3132;               // PPR pulsos por volta
  digitalWrite(M0, LOW);    // Configura modo Micro Passo 1/16
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
  Serial.println(" Micropasso 1/16 ");
}

void P1_32()
{
  PPR = 6400;                // PPR pulsos por volta
  digitalWrite(M0, HIGH);    // Configura modo Micro Passo 1/32
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
  Serial.println(" Micropasso 1/32 ");
}

void TesteMotor()            // Gira motor nos dois sentidos
{
  HOR();
  Serial.print(" / Voltas = ");
  Serial.println(Voltas);
  for (int i = 0; i <= Pulsos; i++)       // Incrementa o Contador
  {
    PASSO();                              // Avança um passo no Motor
  }
  disa_DRV8825();
  delay (2000) ;                           // Atraso de 750 mseg
  ena_DRV8825();
  AHR();
  Serial.print(" / Voltas = ");
  Serial.println(Voltas);
  for (int i = 0; i <= Pulsos; i++)       // Incrementa o Contador
  {
    PASSO();                              // Avança um passo no Motor
  }
  disa_DRV8825();
  delay (2000) ;                           // Atraso de 750 mseg
  ena_DRV8825();
}

void Print_RPM ()
{
  Serial.print(" PPR = ");
  Serial.print(PPR);
  Serial.print(" / Pulsos = ");
  Serial.println(Pulsos);
  Serial.print(" 1/2 T = ");
  Serial.print(MeioPeriodo);
  Serial.print(" us / ");
  Serial.print(" PPS = ");
  Serial.print(PPS, 2);
  Serial.print(" / RPM = ");
  Serial.println(RPM, 2);
}

void loop()
{
  Serial.println();
  Voltas = 1;         // Numero de voltas no Motor
  P1_32();            // Selecione o Modo do Passo FULL() HALF() P1_4() P1_8() P1_16() P1_32()
  Frequencia();       // Calcula RPM
  Print_RPM ();       // Imprime configuracao
  TesteMotor();       // Testa o Motor
}
