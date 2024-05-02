#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>
#include "PololuMagneticEncoder.h"

// Configura pinos
#define LED 2 // LED ESP32

PololuMagneticEncoder encoders;

// Definindo macAddress do ESP
#define MAC {0xD4 , 0x8A , 0xFC , 0xAA , 0x2C , 0xB4}

float ki = 779.6, kp = 38.98;
float erroE = 0, erroD = 0;
float pE = 0, pD = 0;
float iE = 0, iD = 0;
float uE = 0, uD = 0;
float vRefD = 0, vRefE = 0;
float velE, velD;
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

    //Configure o sentido de rotação dos motores aqui
    //--------------------------------------------------
    // Verifica o sentido de rotação do motor direito
    if (vRefD > 0)
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      auxD=true;
    }
    else if (vRefD = 0)
    {
      uD = 0;
      iD = 0;
      auxD=false;
    }
    else
    {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      auxD=true;
    }

    //--------------------------------------------------
    // Verifica o sentido de rotação do motor esquerdo
    if (vRefE > 0)
    {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      auxE=true;
    }
    else if(vRefE = 0)
    {
      uE = 0;
      iE = 0;
      auxE=false;
    }
    else
    {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      auxE=true;
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
      direito = abs(encoders.getCountsAndResetEncoderRight());  
      vRefE = referenceSpeed.left;
      vRefD = referenceSpeed.right;

      // if(vRefD == 0.2 && vRefE == 0.2)
      // {
      //   digitalWrite(LED, !digitalRead(LED));
      // }

      
    }
    

      // Calculo de velocidade
      velE = ((M_PI * 0.042) * (esquerdo/900.0))/0.1;
      velD = ((M_PI * 0.042) * (direito /900.0))/0.1;

      // Calculo do Erro
      erroE = vRefE - velE;
      erroD = vRefD - velD;

      if(auxD)
      {
        // Implemente o controle da roda direita aqui
        iD = iD + erroD * ki * 0.1;
        pD = erroD * kp;
        uD = pD + iD;
        if (uD > 255)
        {
          uD = 255;
        }
        analogWrite(PWMA, uD); 
        auxD=false;
      }

      if(auxE)
      {
        // Implemente o controle da roda esquerda aqui
        iE = iE + erroE * ki * 0.1;
        pE = erroE * kp;
        uE = pE + iE;
        
        if (uE > 255)
        {
          uE = 255;
        }
        analogWrite(PWMB, uE);   
        auxE=false;
      }

      // Atualize as velocidades dos motores aqui

    }
  }     
}
