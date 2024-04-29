#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>
#include <PololuMagneticEncoder.h>


// Definindo macAddress do ESP
#define MAC {0xD4 , 0x8A , 0xFC , 0xC5 , 0xC9 , 0x14}
// D4:8A:FC:C5:C9:14

const int PWMA = 33;
const int AIN1 = 26;
const int AIN2 = 25;
const int STBY = 27;
const int BIN1 = 14;
const int BIN2 = 12;
const int PWMB = 13;

int MTA = 0, MTB = 0;


PololuMagneticEncoder encoders;
long tempo = 0;
int dT = 100;
int esquerdo, direito;
int aux=0;

// Definindo os pinos
#define LED 2

// macAddress do ESP32 TX escolhido
uint8_t broadcastAddress[] = MAC;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int PWM;
    int esquerdo;
    int direito;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message encoderReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status ==0)
    digitalWrite(LED, LOW);
  else
    digitalWrite(LED, HIGH);
}

// Variáveis para armazenamento de dados
int dado;
bool available = false;


// Função callback
void OnDataRecv ( const uint8_t *mac , const uint8_t *incomingData , int len)
{
  memcpy(&dado, incomingData, sizeof(dado));
  available = true;
}

void setup() {

  // Configurando pinos de entrada e saída
  pinMode(LED, OUTPUT);

  // Configurando dispositivo como WiFi Station
  WiFi.mode(WIFI_STA);
  
  // Iniciando ESP-NOW
  if(esp_now_init() != ESP_OK ) 
  {
    digitalWrite(LED, HIGH);
    return ;
  }

  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    digitalWrite(LED, HIGH);
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  encoders.setupEncoders(34, 39, 35, 32);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(STBY, HIGH);

  // Sentido Horario
  // digitalWrite(AIN2, LOW);
  // digitalWrite(AIN1, HIGH);
  // digitalWrite(BIN1, LOW);
  // digitalWrite(BIN2, HIGH);
  // Sentido Anti Horario
  digitalWrite(AIN2, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  Serial.begin(9600);
  delay(10);
}

void loop () {
  // Rotina executada quando um dado é recebido
  if(available)
  {
    available = false;

    aux=1;
  }

  if (millis() - tempo >= dT) {
      tempo = millis();
      if(aux==1){
        esquerdo = encoders.getCountsAndResetEncoderLeft();
        direito = encoders.getCountsAndResetEncoderRight();  
        encoderReadings.PWM = dado;
        encoderReadings.esquerdo = abs(esquerdo);
        encoderReadings.direito = abs(direito);

      // Send message via ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &encoderReadings, sizeof(encoderReadings));
        MTA = dado;
        MTB = dado;
        analogWrite(PWMA, MTA);
        analogWrite(PWMB, MTB);
        aux=0;
      }
    }
}

