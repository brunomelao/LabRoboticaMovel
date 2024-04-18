#include <PololuMagneticEncoder.h>

PololuMagneticEncoder encoders;
long tempo = 0;
int dt = 100;
int esquerdo, direito;

const int PWMA = 33;
const int AIN1 = 26;
const int AIN2 = 25; 
const int STBY = 27; 
const int BIN1 = 14; 
const int BIN2 = 12; 
const int PWMB = 13;

int MTA = 0, MTB = 0; // MTA = Direita MTB = Esquerda


void setup() {
  // put your setup code here, to run once:

  encoders.setupEncoders(34,39,35,32);
  
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(STBY, HIGH);
  Serial.begin(9600);


}

void loop() {
  // put your main code here, to run repeatedly:
  int dado;
  if(Serial.available()>0)
  {
    dado = Serial.read();
    switch (dado)
    {
    case 'w':
      digitalWrite(AIN2, HIGH);
      digitalWrite(AIN1, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      MTA = 255;
      MTB = 255;
      analogWrite(PWMA,MTA);
      analogWrite(PWMB,MTB);
      break;
    case 's':
      digitalWrite(AIN2, LOW);
      digitalWrite(AIN1, HIGH);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      MTA = 120;
      MTB = 120;
      analogWrite(PWMA,MTA);
      analogWrite(PWMB,MTB);
      break;
    case 'c':
      digitalWrite(AIN2, HIGH);
      digitalWrite(AIN1, HIGH);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, HIGH);
      MTA = 120;
      MTB = 120;
      analogWrite(PWMA,MTA);
      analogWrite(PWMB,MTB);
      break;
    case 'a':
      digitalWrite(AIN2, HIGH);
      digitalWrite(AIN1, LOW);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      MTA = 120;
      MTB = 120;
      analogWrite(PWMA,MTA);
      analogWrite(PWMB,MTB);
      break;
    case 'd':
      digitalWrite(AIN2, LOW);
      digitalWrite(AIN1, HIGH);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      MTA = 120;
      MTB = 120;
      analogWrite(PWMA,MTA);
      analogWrite(PWMB,MTB);
      break;
    }
  }

  if (millis() - tempo >= dt)
    {
      tempo = millis();
      // leitura do encoder
      esquerdo = encoders.getCountsAndResetEncoderLeft();
      direito = encoders.getCountsAndResetEncoderRight();
      Serial.print(60*10*esquerdo/900);
      Serial.print(";");
      Serial.println(60*10*direito/900);
    }
}
// 900 pulsos por rotação
// 