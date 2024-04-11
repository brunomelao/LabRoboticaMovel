#include <esp_now.h>
#include <WiFi.h>

#define LED 2

// Definindo macAddress do ESP

#define MAC {0x24 , 0xDC , 0xC3 , 0x45 , 0x0B , 0x68}
//ESP Board MAC Address: D4:8A:FC:C5:C9:14

// macAddress do ESP32 RX escolhido
uint8_t broadcastAddress[] = MAC;

void setup() 
{
  pinMode(LED, OUTPUT);
  
  // Iniciando comunica ção serial
  Serial.begin(115200);

  delay(2000);

  // Colocando ESP em modo WiFi station
  WiFi.mode(WIFI_STA);

  // Iniciando ESP-NOW
  if(esp_now_init() != ESP_OK)
  {
    digitalWrite(LED, HIGH);
    return;
  }

  esp_now_peer_info_t peerInfo;

  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer 
  memset(&peerInfo, 0, sizeof(peerInfo)); 
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    digitalWrite(LED, HIGH);
    return;
  }
}

void loop() 
{
  if (Serial.available() > 0)
  {
    int dado = Serial.read();

    // Status da mensagem
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*) &dado , sizeof(dado));

    if (result == ESP_OK)
    {
      digitalWrite(LED, LOW);
    } 
    else 
    {
      digitalWrite(LED, HIGH);
    }
  }
}
