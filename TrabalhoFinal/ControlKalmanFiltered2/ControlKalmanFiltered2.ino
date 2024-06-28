#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>



#include "Arduino.h"
#include "PololuMagneticEncoder.h"


// Configura pinos
#define LED 2 // LED ESP32
#define pi M_PI
PololuMagneticEncoder encoders;

// Kalman Filter variables
using namespace Eigen;
Matrix<float, 3, 3> P; // Covariance matrix
Matrix<float, 3, 3> Q; // Process noise covariance
Matrix<float, 3, 3> R; // Measurement noise covariance
Matrix<float, 2, 2> M; // 
Matrix<float, 3, 1> X; // State vector

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
float Krho = 0.2;
float Kalpha = 0.5;
float Kbeta = -0.2;

float delta1 = 0.05; //Distancia ate o objetivo [m]
float delta2 = 0.08; //Distancia angular ate objetivo [rad]

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

double ajustaAngulo(double phi){
    phi = fmod(phi, 2*pi);
    if (phi > pi){
        phi = phi - 2*pi;
    }
    return phi;
}

void setupEKF() {
    // Inicializa a matriz de covariância do erro de estimativa
    P = Matrix<float, 3, 3>::Identity();
    
    // Inicializa a matriz de covariância do ruído do processo
    Q = Matrix<float, 3, 3>::Identity() * 0.07;
    
    // Inicializa a matriz de covariância do ruído da medição
    R = Matrix<float, 3, 3>::Identity() * 0.1;
    
    // Inicializa a matriz de covariância do ruído no modelo de entrada
    M = Matrix<float, 2, 2>::Identity() * 0.1;

    // Inicializa o vetor de estado
    X << 0, 0, 0;
}
void EKFupdate(float vel, float wang, float dt2) {
    // Modelo de transição de estado
    float theta = ajustaAngulo(theta + wang * dt2/1000.0);
    Matrix<float, 3, 3> F;
    F << 1, 0, -vel * sin(theta) * dt2/1000.0,
         0, 1,  vel * cos(theta) * dt2/1000.0,
         0, 0, 1;

    Matrix<float, 3, 2> G;
    G << cos(theta) * dt2/1000.0, 0,
         sin(theta) * dt2/1000.0, 0,
         0, dt2/1000.0;

    // Previsão do estado usando o modelo do movimento
    Matrix<float, 3, 1> B;
    B << vel * cos(theta) * dt2/1000.0,
         vel * sin(theta) * dt2/1000.0,
         wang * dt2/1000.0;
    
    // Estado previsto
    X += B;

    // Previsão da matriz de covariância do erro de estimativa
    P = F * P * F.transpose() + G * M * G.transpose() + Q;

    // Modelo de medição
    Matrix<float, 3, 3> H;
    H << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    // Medição
    Matrix<float, 3, 1> Z;
    Z << posX,
         posY,
         posTheta;

    // Calculo da diferença da medição
    Matrix<float, 3, 1> y = Z - H * X;

    // Calculo da matriz de inovação
    Matrix<float, 3, 3> S = H * P * H.transpose() + R;

    // Calculo do ganho de Kalman
    Matrix<float, 3, 3> K = P * H.transpose() * S.inverse();

    // Atualização do estado
    X += K * y;

    // Atualização da matriz de covariância do erro de estimativa
    P = (Matrix<float, 3, 3>::Identity() - K * H) * P;

    // Normaliza o ângulo
    X(2) = ajustaAngulo(X(2));
}
void setup() {
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

    setupEKF();
}

void loop() {
    if(aux == 0){
        goalX = 1.0;
        goalY = 1.0;
        goalTheta = 0;
    } else if(aux == 1) {
        goalX = 0;
        goalY = 1;
        goalTheta = 0;
    } else if(aux == -1) {
        goalX = 0.6;
        goalY = 0.95;
        goalTheta = 1.8;
    }

    vRefD = v + w * l;
    vRefE = v - w * l;

    dx = goalX - posX;
    dy = goalY - posY;
    dtheta = ajustaAngulo(goalTheta - posTheta);

    // Coordenadas Polares
    rho = sqrt(dx*dx + dy*dy);
    gama = ajustaAngulo(atan2(dy, dx + 0.0001));

    alpha = ajustaAngulo(gama - posTheta);
    beta = ajustaAngulo(goalTheta - gama);

    while(rho > delta1 || abs(alpha) > delta2 || abs(beta) > delta2){
        dx = goalX - posX;
        dy = goalY - posY;
        dtheta = ajustaAngulo(goalTheta - posTheta);

        rho = sqrt(dx*dx + dy*dy);
        gama = ajustaAngulo(atan2(dy, dx + 0.00001));
        alpha = ajustaAngulo(gama - posTheta);
        beta = ajustaAngulo(goalTheta - gama);

        v = min(Krho * rho, vmax);

        if(abs(alpha) > pi/2){
            v = -v;
            alpha = ajustaAngulo(alpha + pi);
            beta = ajustaAngulo(beta + pi);
        }

        w = Kalpha * alpha + Kbeta * beta;
        if(w != 0) {
            w = (w / abs(w)) * min(abs(w), wmax);
        }

        vRefD = v + w * l;
        vRefE = v - w * l;

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
        } else if(vRefE == 0) {
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

        if (millis() - tempo >= dt) {
            tempo = millis();

            esquerdo = encoders.getCountsAndResetEncoderLeft();
            direito = encoders.getCountsAndResetEncoderRight(); 

            velE = 0.0015 * esquerdo;
            velD = 0.0015 * direito;

            erroE = vRefE - velE;
            erroD = vRefD - velD;

            if(auxD) {
                iD = iD + erroD * kiDir * 0.1;
                pD = erroD * kpDir;
                uD = abs(pD) + abs(iD);
                uD = (int)uD;

                if (uD > 150) {
                    uD = 150;
                } else if(uD < 0) {
                    uD = 0;
                }
            }

            if(auxE) {
                iE = iE + erroE * kiEsq * 0.1;
                pE = erroE * kpEsq;
                uE = abs(pE) + abs(iE);
                uE = (int)uE;

                if (uE > 150) {
                    uE = 150;
                } else if(uE < 0) {
                    uE = 0;
                }
            }

            v = (velE + velD) / 2;
            w = (-velE + velD) / (2 * l);
            
            EKFupdate(v, w, dt);
            Serial.print("Pos: (");
            Serial.print(posX);
            Serial.print(", ");
            Serial.print(posY);
            Serial.print(", ");
            Serial.print(posTheta);
            Serial.println();

            posX = X(0);
            posY = X(1);
            posTheta = X(2);

            Serial.print("Pos: (");
            Serial.print(posX);
            Serial.print(", ");
            Serial.print(posY);
            Serial.print(", ");
            Serial.print(posTheta);
            Serial.print(") | Goal: (");
            Serial.print(goalX);
            Serial.print(", ");
            Serial.print(goalY);
            Serial.print(", ");
            Serial.print(goalTheta);
            Serial.print(") | Erro: (");
            Serial.print(erroE);
            Serial.print(", ");
            Serial.print(erroD);
            Serial.print(") | Vel: (");
            Serial.print(velE);
            Serial.print(", ");
            Serial.print(velD);
            Serial.print(") | rho, alpha e beta: (");
            Serial.print(rho);
            Serial.print(", ");
            Serial.print(alpha);
            Serial.print(", ");
            Serial.print(beta);
            Serial.print(")");
            Serial.println();

            analogWrite(PWMA, uD); 
            analogWrite(PWMB, uE);   
        }
    }

    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);

    aux = aux + 1;

    if(aux == 2) {
        digitalWrite(LED, LOW);
    } else if(aux == 1) {
        digitalWrite(LED, HIGH);
    } else if(aux == 3) {
        digitalWrite(LED, HIGH);
    }
}
