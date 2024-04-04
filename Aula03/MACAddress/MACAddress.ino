
#include <WiFi.h>

void setup(){

  delay(2000);

  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP Board MAC Address: ");
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}


