#include <esp_now.h>
#include <WiFi.h>

#define LED 2

// Definindo macAddress do ESP
#define MAC {0x24 , 0xDC , 0xC3 , 0x45 , 0x0B , 0x68}
// 24:DC:C3:45:0B:68

// macAddress do ESP32 RX escolhido
uint8_t broadcastAddress[] = MAC;

esp_now_peer_info_t peerInfo;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int PWM;
    int esquerdo;
    int direito;
} struct_message;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  if (status == 0)
    digitalWrite(LED, LOW);
  else
    digitalWrite(LED, HIGH);
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.println(incomingReadings.PWM);
  Serial.println(incomingReadings.esquerdo);
  Serial.println(incomingReadings.direito);
}

void setup() 
{
  pinMode(LED, OUTPUT);
  
  // Iniciando comunica ção serial
  Serial.begin(9600);

  delay(2000);

  // Colocando ESP em modo WiFi station
  WiFi.mode(WIFI_STA);

  // Iniciando ESP-NOW
  if(esp_now_init() != ESP_OK)
  {
    digitalWrite(LED, HIGH);
    return;
  }

  esp_now_register_send_cb(OnDataSent);

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

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() 
{
  if (Serial.available() > 0)
  {
    int dado = Serial.read();

    // Status da mensagem
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*) &dado , sizeof(dado));

    if (result == ESP_OK)
      digitalWrite(LED, LOW);
    else 
      digitalWrite(LED, HIGH);
  }
}

