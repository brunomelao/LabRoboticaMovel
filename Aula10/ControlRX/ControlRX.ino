#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>
#include "PololuMagneticEncoder.h"

// Configura pinos
#define LED 2 // LED ESP32

PololuMagneticEncoder encoders;

// Definindo macAddress do ESP
#define MAC {0xD4 , 0x8A , 0xFC , 0xC5 , 0xC9 , 0x14}

float kiEsq = 529.5046, kpEsq = 26.4752;
float kiDir = 510.9769, kpDir = 25.3988;
float erroE = 0, erroD = 0;
float pE = 0, pD = 0;
float iE = 0, iD = 0;
int uE = 0, uD = 0;
float vRefD = 0, vRefE = 0;
float velE, velD;


#define PWMA 33
#define AIN1 26
#define AIN2 25
#define STBY 27
#define BIN1 14
#define BIN2 12
#define PWMB 13

// macAddress do ESP32 TX escolhido
uint8_t MAC_TX[] = MAC;

int dt = 100; // Tempo de amostragem
long tempo = 0;
bool aux = false; // Variável auxiliar

int esquerdo = 0, direito = 0; // Variáveis para armazenar leitura do encoder

bool auxD = false, auxE = false;

typedef struct receive_message {
    float left;
    float right;
} receive_message;

// Create a struct_message called BME280Readings to hold sensor readings
receive_message referenceSpeed;

typedef struct send_message {
    int esquerdo;
    int direito;
} send_message;

// Create a struct_message called BME280Readings to hold sensor readings
send_message encoderRead;

bool available = false;

// Função callback
void OnDataRecv(const uint8_t *mac , const uint8_t *incomingData , int len)
{
  memcpy(&referenceSpeed, incomingData, sizeof(referenceSpeed));
  available = true;
  aux = true;
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

  // Configurando dispositivo como WiFi Station
  WiFi.mode(WIFI_STA);
  
  // Iniciando ESP-NOW
  if(esp_now_init() != ESP_OK ) 
  {
    return ;
  }

  esp_now_peer_info_t peerInfo;

  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer 
  memset(&peerInfo, 0, sizeof(peerInfo)); 
  memcpy(peerInfo.peer_addr, MAC_TX, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    return;
  }

  // Configurando função de CallBack
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  if (available) 
  {
    available = false;
    
    vRefE = referenceSpeed.left;
    vRefD = referenceSpeed.right;
    //Configure o sentido de rotação dos motores aqui
    //--------------------------------------------------
    // Verifica o sentido de rotação do motor direito
    if (vRefD > 0)
    {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      auxD=true;
      kiDir=510.9769;
      kpDir=25.3988;
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
      kiDir=514.7678;
      kpDir=25.7384;
    }

    //--------------------------------------------------
    // Verifica o sentido de rotação do motor esquerdo
    if (vRefE > 0)
    {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      auxE=true;
      kiEsq=529.5046;
      kpEsq=26.4752;

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
      kiEsq=505.146;
      kpEsq=25.2573;
    }
    //--------------------------------------------------

    encoderRead.esquerdo = esquerdo;
    encoderRead.direito = direito;

    esp_now_send(MAC_TX, (uint8_t *) &encoderRead, sizeof(encoderRead));
  }

  // Leitura do encoder a cada dt milissegundos
  if (millis() - tempo >= dt) 
  {
    tempo = millis();

    if(aux)
    {
      // Faça a leitura dos encoders aqui
      esquerdo = abs(encoders.getCountsAndResetEncoderLeft());
      direito =  abs(encoders.getCountsAndResetEncoderRight()); 
      
      // if(vRefD == 0.2 && vRefE == 0.2)
      // {
      //   digitalWrite(LED, !digitalRead(LED));
      // }

      // Calculo de velocidade
      velE = 0.0015*esquerdo;
      velD = 0.0015*direito;

      // Calculo do Erro
      erroE = vRefE - velE;
      erroD = vRefD - velD;

      if(auxD)
      {
        // Implemente o controle da roda direita aqui
        iD = iD + erroD * kiDir * 0.1;
        pD = erroD * kpDir;
        uD = pD + iD;
        uD= (int)uD;

        if (uD > 255)
        {
          uD = 255;
        }
        else if(uD < 0){
          uD=0;
        }
        digitalWrite(LED, !digitalRead(LED));
   
      }

      if(auxE)
      {
        // Implemente o controle da roda esquerda aqui
        iE = iE + erroE * kiEsq * 0.1;
        pE = erroE * kpEsq;
        uE = pE + iE;
        uE= (int)uE;
        
        
        if (uE > 255)
        {
          uE = 255;
        }
        else if(uE < 0){
          uE=0;
        }

        
      }

      // Atualize as velocidades dos motores aqui
      analogWrite(PWMA, uD); 
      analogWrite(PWMB, uE);   


    }
  }
      
}
