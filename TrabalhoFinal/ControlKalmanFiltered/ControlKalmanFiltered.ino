#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>
#include "PololuMagneticEncoder.h"
#include <Kalman.h>

// Configura pinos
#define LED 2 // LED ESP32
#define pi M_PI
PololuMagneticEncoder encoders;

float kiEsq = 529.5046, kpEsq = 26.4752;
float kiDir = 510.9769, kpDir = 25.3988;
float erroE = 0, erroD = 0;
float pE = 0, pD = 0;
float iE = 0, iD = 0;
int uE = 0, uD = 0;
float vRefD = 0, vRefE = 0;
float velE, velD;



//Definicoes do controlador de posicao
float r = 0.021;
float l = 0.0425;

//Ganhos do controlador de posicao
//(Krho > 0, Kalpha > Krho, Kbeta < 0)  
float Krho = 0.20;    
float Kalpha = 0.5;   
float Kbeta = -0.21;

float delta1 = 0.03;     //Distancia ate o objetivo [m]
float delta2 = 0.05;  //Distancia angular ate objetivo [rad]

float vmax = 0.7; // [m/s]
float wmax = pi; // [m/s]

float v = 0.5; // [m/s]
float w = 0; // [rad/s]

float dx = 0;
float dy = 0;
float dtheta = 0;

float rho = 0;
double gama = 0;
double beta = 0;
double alpha = 0;

float posX=0, posY=0, posTheta=0;
float goalX=1, goalY=0.00, goalTheta=0;
float z[3];

#define PWMA 33
#define AIN1 26
#define AIN2 25
#define STBY 27
#define BIN1 14
#define BIN2 12
#define PWMB 13

// Definindo os filtros de Kalman para x, y e theta
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanTheta;

int dt = 100; // Tempo de amostragem
long tempo = 0;
int aux = 0; // Variável auxiliar

int esquerdo = 0, direito = 0; // Variáveis para armazenar leitura do encoder

bool auxD = false, auxE = false;

double ajustaAngulo(double phi){
  phi=fmod(phi,2*pi);
  if (phi>pi){
    phi=phi-2*pi;
  }
  return phi;
}

void setup()
{
  // Configura pinos como saída
  pinMode(LED,OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(STBY, HIGH);

  encoders.setupEncoders(35,32,34,39);
  Serial.begin(9600);
  aux=0;
  // Inicialização dos filtros de Kalman
  kalmanX.setAngle(0); // Inicializa o ângulo estimado do filtro de Kalman
  kalmanY.setAngle(0);
  kalmanTheta.setAngle(0);

  // Ajuste dos parâmetros do filtro de Kalman
  kalmanX.setQangle(0.07);
  kalmanX.setQbias(0.03);
  kalmanX.setRmeasure(0.5);

  kalmanY.setQangle(0.18);
  kalmanY.setQbias(0.03);
  kalmanY.setRmeasure(0.5);

  kalmanTheta.setQangle(0.13);
  kalmanTheta.setQbias(0.03);
  kalmanTheta.setRmeasure(0.5);
}

void loop()
{
  if(aux==0){
    goalX=1.0;
    goalY=0.0;
    goalTheta=0;
  }
  if(aux==1){// Ponto (0,1,0)
    // Caso saia de (1,0,0) foi pra frente
    goalX=0.00;
    goalY=1;
    goalTheta=0.;
    // Caso saia de (0,0,0)

  }
  if(aux==-1){// Ponto (1,1)
    // Caso saia de (1,0,0) foi pra frente
    goalX=0.6;
    goalY=0.95;
    goalTheta=1.8;
    // Caso saia de (0,1,0) foi para esquerda

  }
  vRefD = v + w*l;
  vRefE = v - w*l;

  dx = goalX - posX;
  dy = goalY - posY;
  dtheta =ajustaAngulo(goalTheta - posTheta);
  
  // Coordenadas Polares
  rho=sqrt(dx*dx + dy*dy);
  gama = ajustaAngulo(atan2(dy,dx+0.001));

  alpha = ajustaAngulo(gama - posTheta);
  //beta: angulo entre a posicao do robo e o objetivo
  beta = ajustaAngulo(goalTheta - gama);

  //Configure o sentido de rotação dos motores aqui
  //--------------------------------------------------
  //Verifica o sentido de rotação do motor direito
  if (vRefD > 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    auxD=true;
    kpDir=25.3988;
    kiDir=510.9769;
  }
  else if (vRefD == 0)
  {
    uD = 0;
    iD = 0;
    auxD=false;
  }
  else
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    vRefD=abs(vRefD);
    auxD=true;
    kpDir=25.7384;
    kiDir=514.7678;
  }

  //--------------------------------------------------
  // Verifica o sentido de rotação do motor esquerdo
  if (vRefE > 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    auxE=true;
    kpEsq=26.4752;
    kiEsq=529.5046;

  }
  else if(vRefE == 0)
  {
    uE = 0;
    iE = 0;
    auxE=false;
  }
  else
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    vRefE=abs(vRefE);
    auxE=true;
    kpEsq=25.2573;
    kiEsq=505.146;
  }
  //--------------------------------------------------

  while (rho > delta1 || abs(alpha)>delta2 || abs(beta)>delta2){
    dx = goalX - posX;
    dy = goalY - posY;
    dtheta =ajustaAngulo(goalTheta - posTheta);
    
    // Coordenadas Polares
    rho=sqrt(dx*dx + dy*dy);
    gama = ajustaAngulo(atan2(dy,dx+0.0001));

    //alpha: angulo entre a frente do robo e o objetivo
    alpha = ajustaAngulo(gama - posTheta);
    //beta: angulo entre a posicao do robo e o objetivo
    beta = ajustaAngulo(goalTheta - gama);
    // Serial.print("rho: ");
    // Serial.print(rho);
    // Serial.print(" ||  gamma:");
    // Serial.print(gama);
    // Serial.print(" ||  alpha:");
    // Serial.print(alpha);
    // Serial.print(" ||  beta:");
    // Serial.println(beta);
    v = min(Krho*rho,vmax);
    
    if(abs(alpha)>pi/2){
      v=-v;
      alpha=ajustaAngulo(alpha+pi);
      beta=ajustaAngulo(beta+pi);
    }

    w = Kalpha*alpha + Kbeta*beta;
    if(w!=0)
    {
      w = (w/abs(w))*min(abs(w),wmax);
    }

    vRefD = v + w*l;
    vRefE = v - w*l;

    //Verifica o sentido de rotação do motor direito
    if (vRefD > 0)
    {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      auxD=true;
      kpDir=25.3988;
      kiDir=510.9769;
    }
    else if (vRefD == 0)
    {
      uD = 0;
      iD = 0;
      auxD=false;
    }
    else
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      //vRefD=abs(vRefD);
      auxD=true;
      kpDir=25.7384;
      kiDir=514.7678;
    }

    //--------------------------------------------------
    // Verifica o sentido de rotação do motor esquerdo
    if (vRefE > 0)
    {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      auxE=true;
      kpEsq=26.4752;
      kiEsq=529.5046;

    }
    else if(vRefE == 0)
    {
      uE = 0;
      iE = 0;
      auxE=false;
    }
    else
    {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      //vRefE=abs(vRefE);
      auxE=true;
      kpEsq=25.2573;
      kiEsq=505.146;
    }
    
  // Leitura do encoder a cada dt milissegundos
    if (millis() - tempo >= dt) {
      tempo = millis();

      // Faça a leitura dos encoders aqui
      esquerdo = encoders.getCountsAndResetEncoderLeft();
      direito = encoders.getCountsAndResetEncoderRight();

      // Calculo de velocidade
      velE = 0.0015 * esquerdo;
      velD = 0.0015 * direito;

      // Calculo do Erro
      erroE = vRefE - velE;
      erroD = vRefD - velD;

      if (auxD) {
        // Controle da roda direita
        iD = iD + erroD * kiDir * 0.1;
        pD = erroD * kpDir;
        uD = abs(pD) + abs(iD);
        uD = (int)uD;
        if (uD > 150) {
          uD = 150;
        } else if (uD < 0) {
          uD = 0;
        }
      }

      if (auxE) {
        // Controle da roda esquerda
        iE = iE + erroE * kiEsq * 0.1;
        pE = erroE * kpEsq;
        uE = abs(pE) + abs(iE);
        uE = (int)uE;
        if (uE > 150) {
          uE = 150;
        } else if (uE < 0) {
          uE = 0;
        }
      }

      v = (velE + velD) / 2;
      w = (-velE + velD) / (2 * l);

      // Atualize as medições para o filtro de Kalman
      z[0] = posX + v * cos(posTheta) * dt / 1000.0;
      z[1] = posY + v * sin(posTheta) * dt / 1000.0;
      z[2] = ajustaAngulo(posTheta + w * dt / 1000.0);

      Serial.print("Pos: (");
      Serial.print(z[0]);
      Serial.print(", ");
      Serial.print(z[1]);
      Serial.print(", ");
      Serial.print(z[2]);
      Serial.print(") | ");
      // Aplica o filtro de Kalman
      float posX_kalman = kalmanX.getAngle(z[0], v * cos(posTheta), dt / 1000.0);
      float posY_kalman = kalmanY.getAngle(z[1], v * sin(posTheta), dt / 1000.0);
      float posTheta_kalman = kalmanTheta.getAngle(z[2], w, dt / 1000.0);

      // Atualiza a posição estimada
      posX = posX_kalman;
      posY = posY_kalman;
      posTheta = posTheta_kalman;

      Serial.print("Pos: (");
      Serial.print(posX);
      Serial.print(", ");
      Serial.print(posY);
      Serial.print(", ");
      Serial.print(posTheta);
      Serial.print(")");
      Serial.println();
      // Serial.print(goalX);
      // Serial.print(", ");
      // Serial.print(goalY);
      // Serial.print(", ");
      // Serial.print(goalTheta);
      // Serial.print(") | Erro: (");
      // Serial.print(erroE);
      // Serial.print(", ");
      // Serial.print(erroD);
      // Serial.print(") | Vel: (");
      // Serial.print(velE);
      // Serial.print(", ");
      // Serial.print(velD);
      // Serial.print(") | rho, alpha e beta: (");
      // Serial.print(rho);
      // Serial.print(", ");
      // Serial.print(alpha);
      // Serial.print(", ");
      // Serial.print(beta);
      // Serial.print(")");
      // Serial.println();

      // Atualize as velocidades dos motores aqui
      analogWrite(PWMA, uD);
      analogWrite(PWMB, uE);
    }
  }


  // Para o robô
  digitalWrite(AIN2, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0); 
  analogWrite(PWMB, 0); 
  
  // Proxima posição de destino
  aux=aux+1;
  // Indicador de posição alcançada
  if(aux==2)
  {
    digitalWrite(LED, LOW);
  }
  else if(aux==1)
  {
    digitalWrite(LED, HIGH);
  }
  else if(aux==-10)
  {
    digitalWrite(LED, HIGH);
  }
}