#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>
#include "PololuMagneticEncoder.h"

// Configura pinos
#define LED 2 // LED ESP32
#define pi M_PI
PololuMagneticEncoder encoders;

// Controlador PID
float kiEsq = 529.5046, kpEsq = 26.4752;
float kiDir = 510.9769, kpDir = 25.3988;
float erroE = 0, erroD = 0;
float pE = 0, pD = 0;
float iE = 0, iD = 0;
int uE = 0, uD = 0;
float vRefD = 0, vRefE = 0;
float velE, velD;

// Controlador de posição
float r = 0.021;
float l = 0.0425;

// Ganhos do controlador de posição
float Krho = 0.2;
float Kalpha = 0.52;
float Kbeta = -0.2;

float delta1 = 0.05; // Distancia até o objetivo [m]
float delta2 = 0.08; // Distancia angular até objetivo [rad]

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
float goalX=1.05, goalY=0.05, goalTheta=0;

#define PWMA 33
#define AIN1 26
#define AIN2 25
#define STBY 27
#define BIN1 14
#define BIN2 12
#define PWMB 13

int dt = 100; // Tempo de amostragem
long tempo = 0;
int aux = 0; // Variável auxiliar

int esquerdo = 0, direito = 0; // Variáveis para armazenar leitura do encoder

bool auxD = false, auxE = false;

// Filtro de Kalman - Inicialização
float sigma_z_x = 0.1; // [m]
float sigma_z_y = 0.1; // [m]

// Matriz de covariância do sensor
float R[2][2] = {
  {sigma_z_x * sigma_z_x, 0},
  {0, sigma_z_y * sigma_z_y}
};

// Matriz de observação
float H[2][3] = {
  {1, 0, 0},
  {0, 1, 0}
};

// Posição estimada pelo filtro de Kalman
float P_Hodometria[3] = {
  posX, // x inicial do robô [m]
  posY, // y inicial do robô [m]
  posTheta // th inicial do robô [rad]
};

// Matriz de covariância da estimativa
float P[3][3] = {{0.0}};

// Função para ajustar o ângulo entre -pi e pi
double ajustaAngulo(double phi){
  phi = fmod(phi, 2 * pi);
  if (phi > pi){
    phi = phi - 2 * pi;
  }
  return phi;
}

void setup() {
  // Configura pinos como saída
  pinMode(LED, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(STBY, HIGH);

  encoders.setupEncoders(35, 32, 34, 39);
  Serial.begin(9600);
  aux = 0;
}

void loop() {
  if (aux == 0) {
    goalX = 1.0;
    goalY = 1.0;
    goalTheta = 0;
  }

  // Atualiza as referências de velocidade
  vRefD = v + w * l;
  vRefE = v - w * l;

  dx = goalX - posX;
  dy = goalY - posY;
  dtheta = ajustaAngulo(goalTheta - posTheta);

  // Coordenadas Polares
  rho = sqrt(dx * dx + dy * dy);
  gama = ajustaAngulo(atan2(dy, dx));

  alpha = ajustaAngulo(gama - posTheta);
  beta = ajustaAngulo(goalTheta - gama);

  while (rho > delta1 || abs(alpha) > delta2 || abs(beta) > delta2) {
    // Atualiza as referências de velocidade
    v = min(Krho * rho, vmax);
    if (abs(alpha) > pi / 2) {
      v = -v;
      alpha = ajustaAngulo(alpha + pi);
      beta = ajustaAngulo(beta + pi);
    }

    w = Kalpha * alpha + Kbeta * beta;
    if (w != 0) {
      w = (w / abs(w)) * min(abs(w), wmax);
    }

    vRefD = v + w * l;
    vRefE = v - w * l;

    // Controle dos motores
    if (vRefD > 0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      auxD = true;
      kpDir = 25.3988;
      kiDir = 510.9769;
    } else if (vRefD == 0) {
      uD = 0;
      iD = 0;
      auxD = false;
    } else {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      vRefD = abs(vRefD);
      auxD = true;
      kpDir = 25.7384;
      kiDir = 514.7678;
    }

    if (vRefE > 0) {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      auxE = true;
      kpEsq = 26.4752;
      kiEsq = 529.5046;
    } else if (vRefE == 0) {
      uE = 0;
      iE = 0;
      auxE = false;
    } else {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      vRefE = abs(vRefE);
      auxE = true;
      kpEsq = 25.2573;
      kiEsq = 505.146;
    }

    // Leitura do encoder a cada dt milissegundos
    if (millis() - tempo >= dt) {
      tempo = millis();

      // Leitura dos encoders
      esquerdo = encoders.getCountsAndResetEncoderLeft();
      direito = encoders.getCountsAndResetEncoderRight();

      // Cálculo de velocidade
      velE = 0.0015 * esquerdo;
      velD = 0.0015 * direito;

      // Cálculo do erro
      erroE = vRefE - velE;
      erroD = vRefD - velD;

      if (auxD) {
        // Controle PID para a roda direita
        iD = iD + erroD * kiDir * 0.1;
        pD = erroD * kpDir;
        uD = abs(pD) + abs(iD);
        uD = (int)uD;
        if (uD > 150) {
          uD = 150;
        }
      }

      if (auxE) {
        // Controle PID para a roda esquerda
        iE = iE + erroE * kiEsq * 0.1;
        pE = erroE * kpEsq;
        uE = abs(pE) + abs(iE);
        uE = (int)uE;
        if (uE > 150) {
          uE = 150;
        }
      }

      // Atualiza a posição estimada
      float dPdt_Robo[3] = {
        v * cos(posTheta),
        v * sin(posTheta),
        w
      };

      for (int i = 0; i < 3; i++) {
        P_Hodometria[i] += dPdt_Robo[i] * (dt / 1000.0);
      }
      P_Hodometria[2] = ajustaAngulo(P_Hodometria[2]);

      // Leitura do sensor (sem ruído)
      float y[2] = {
        P_Hodometria[0],
        P_Hodometria[1]
      };

      // Filtro de Kalman - Atualização
      float S[2][2] = {{0.0}};
      float K[3][2] = {{0.0}};
      float y_hat[2] = {0.0};

      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
          for (int k = 0; k < 3; k++) {
            S[i][j] += H[i][k] * P[k][k] * H[j][k];
          }
          S[i][j] += R[i][j];
        }
      }

      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
          for (int k = 0; k < 2; k++) {
            K[i][j] += P[i][k] * H[j][k] * S[j][j];
          }
        }
      }

      for (int i = 0; i < 2; i++) {
        y_hat[i] = y[i] - (H[i][0] * P_Hodometria[0] + H[i][1] * P_Hodometria[1] + H[i][2] * P_Hodometria[2]);
      }

      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
          P_Hodometria[i] += K[i][j] * y_hat[j];
        }
      }

      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          P[i][j] -= K[i][j] * H[j][j] * P[j][j];
        }
      }

      posX = P_Hodometria[0];
      posY = P_Hodometria[1];
      posTheta = ajustaAngulo(P_Hodometria[2]);

      // Atualiza os motores
      analogWrite(PWMA, uD);
      analogWrite(PWMB, uE);
    }

    // Encerra a navegação ao atingir o objetivo
    dx = goalX - posX;
    dy = goalY - posY;
    dtheta = ajustaAngulo(goalTheta - posTheta);

    rho = sqrt(dx * dx + dy * dy);
    gama = ajustaAngulo(atan2(dy, dx));

    alpha = ajustaAngulo(gama - posTheta);
    beta = ajustaAngulo(goalTheta - gama);
  }

  if (aux == 0) {
    aux = 1;
  } else {
    aux = 0;
  }
}
