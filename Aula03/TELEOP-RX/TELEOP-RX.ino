#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>

// Definindo macAddress do ESP
#define MAC {0xD4 , 0x8A , 0xFC , 0xC5 , 0xC9 , 0x14}

//ESP Board MAC Address: 24:DC:C3:45:0B:68

// Definindo os pinos
#define LED 2

// Variáveis constantes e inteiras
// ROBO B
#define PWMA 33
#define AIN1 26
#define AIN2 25
#define STBY 27
#define BIN1 14
#define BIN2 12
#define PWMB 13

// Variáveis globais inteiras
int MTA = 0;
int MTB = 0;


// macAddress do ESP32 TX escolhido
uint8_t mastermacAddress[] = MAC;

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
  pinMode(PWMA,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(STBY,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  //Configurando STBY com valor lógico alto
  digitalWrite(STBY,HIGH);
  //Configurando a comunicação serial
  Serial.begin(9600);


  // Configurando dispositivo como WiFi Station
  WiFi.mode(WIFI_STA);
  
  // Iniciando ESP-NOW
  if(esp_now_init() != ESP_OK ) 
  {
    digitalWrite(LED, HIGH);
    return ;
  }
  
  // Configurando função de CallBack
  esp_now_register_recv_cb(OnDataRecv);
}

void loop () {
  // Rotina executada quando um dado é recebido
  if(available)
  {
    available = false;

    if(dado == 30  || dado == 119){
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);
    MTA = 50;
    MTB = 50;
    analogWrite(PWMA,MTA);
    analogWrite(PWMB,MTB);
  }
  //Caso o dado for igual a b, mover os motores no sentido contrário
  if(dado == 31 || dado == 115){
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
    MTA = 50;
    MTB = 50;
    analogWrite(PWMA,MTA);
    analogWrite(PWMB,MTB);
  }
  //Caso o dado for igual a c, parar os motores
  if(dado == 32){
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,LOW);
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,LOW);
    MTA = 0;
    MTB = 0;
    analogWrite(PWMA,MTA);
    analogWrite(PWMB,MTB);
  }
  //Caso o dado for igual a d, gira para a direita
  if(dado == 28 || dado == 97 ){
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
    MTA = 50;
    MTB = 50;
    analogWrite(PWMA,MTA);
    analogWrite(PWMB,MTB);
  }
  //Caso o dado for igual a e, gira para a esquerda
  if(dado == 29 || dado == 100){
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);
    MTA = 50;
    MTB = 50;
    analogWrite(PWMA,MTA);
    analogWrite(PWMB,MTB);
  }


    
  }
}
