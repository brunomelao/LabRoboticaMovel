#include <esp_now.h>
#include <WiFi.h>

#define LED 2

// Definindo macAddress do ESP
#define MAC {0x24 , 0xDC , 0xC3 , 0x45 , 0xD0 , 0x60}

// macAddress do ESP32 RX escolhido
uint8_t MAC_RX[] = MAC;

typedef struct message {
    int left;
    int right;
} message;

// Create a struct_message to hold incoming sensor readings
message encodersRead;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&encodersRead, incomingData, sizeof(encodersRead));
  Serial.println(encodersRead.left);
  Serial.println(encodersRead.right);
}

byte msg[9];

void setup() 
{
  pinMode(LED, OUTPUT);
  
  // Iniciando comunica ção serial
  Serial.begin(9600);

  // Colocando ESP em modo WiFi station
  WiFi.mode(WIFI_STA);

  // Iniciando ESP-NOW
  if(esp_now_init() != ESP_OK)
  {
    return;
  }

  esp_now_peer_info_t peerInfo;

  peerInfo.channel = 0;  
  peerInfo.encrypt = false; 
  memset(&peerInfo, 0, sizeof(peerInfo)); 
  memcpy(peerInfo.peer_addr, MAC_RX, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
    return;

  esp_now_register_recv_cb(OnDataRecv);

  delay(2000);
}

void loop() 
{
  if (Serial.available() > 8) // Observe que o código só entrará no if depois de ler 9 bytes
  {    
    // Fazer a leitura da mensagem de 9 bytes enviada pelo Matlab aqui
    
    // Verificar se o primeiro elemento da mensagem é igual a 1

    // Em caso afirmativo, utilizar o comando abaixo para enviar a mensagem para o robô
    msg[0] = Serial.read();
    for(int i=1; i<9; i++)
    {
      msg[i] = Serial.read();
    }   

    if  msg[0] == 1)
    {      
      esp_now_send(MAC_RX, (uint8_t*) &msg[1] , sizeof(float)*2);
    }
  } 

}
