const int PWMA = 33, AIN1 = 26, AIN2 = 25, STBY = 27, BIN1 = 14, BIN2 = 12, PWMB = 13;

int MTA = 0, MTB = 0; // MTA = Direita MTB = Esquerda

void setup() {
  // put your setup code here, to run once:
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
// PINOS AIN1 e BIN1 quando LOW: para frente, AIN2 e BIN2 quando LOW: para trás, tudo HIGH trava as rodas

void loop() {
  // put your  main code here, to run repeatedly:
  int dado;
  if(Serial.available()>0)
  {
    dado = Serial.read();
    switch (dado)
    {
    case 'a':
      digitalWrite(AIN2, HIGH);
      digitalWrite(AIN1, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      MTA = 255;
      MTB = 255;
      analogWrite(PWMA,MTA);
      analogWrite(PWMB,MTB);
      break;
    case 'b':
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
    case 'd':
      digitalWrite(AIN2, HIGH);
      digitalWrite(AIN1, LOW);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      MTA = 120;
      MTB = 120;
      analogWrite(PWMA,MTA);
      analogWrite(PWMB,MTB);
      break;
    case 'e':
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
}




