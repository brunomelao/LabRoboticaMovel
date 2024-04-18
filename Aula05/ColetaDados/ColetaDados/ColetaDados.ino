#include <PololuMagneticEncoder.h>

PololuMagneticEncoder encoders;
long tempo = 0;
int dT = 100;
int esquerdo, direito;


const int PWMA = 33;
const int AIN1 = 26;
const int AIN2 = 25;
const int STBY = 27;
const int BIN1 = 14;
const int BIN2 = 12;
const int PWMB = 13;

int MTA = 0, MTB = 0;

int dado = 0;
int aux = 0;

void setup() {
  // put your setup code here, to run once:

  encoders.setupEncoders(34, 39, 35, 32);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(STBY, HIGH);

  digitalWrite(AIN2, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  Serial.begin(9600);
  delay(10);
  Serial.println('a');
  ///char data = 'b';
  //while (data != 'a') {
    //data = Serial.read();
  //}
}


void loop() {
  // put your main code here, to run repeatedly:
  //
   if(Serial.available() > 0)
  {

    aux = 1;
    dado = Serial.read();
  }
    if (millis() - tempo >= dT) {
      tempo = millis();
      if (aux == 1) {
        esquerdo = encoders.getCountsAndResetEncoderLeft();
        direito = encoders.getCountsAndResetEncoderRight();  
        Serial.println(dado);
        Serial.println(abs(esquerdo));
        Serial.println(abs(direito));
        MTA = dado;
        MTB = dado;
        analogWrite(PWMA, MTA);
        analogWrite(PWMB, MTB);
      }
    }
  }

//}