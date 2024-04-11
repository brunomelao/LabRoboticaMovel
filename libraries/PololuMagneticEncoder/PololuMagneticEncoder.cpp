/*
  PololuMagneticEncoder.cpp - Library for using Pololu Magnetic Encoders
  Author: Ana Sophia Cavalcanti Alves Vilas Boas
  Creation date: April 2023
*/

#include "Arduino.h"
#include "PololuMagneticEncoder.h"

int PololuMagneticEncoder::pin[4] = {0, 0, 0, 0}; // Inicializa o atributo estático pin
volatile long PololuMagneticEncoder::countsEncoderLeft = 0; // Inicializa o atributo estático countsEncoderLeft
volatile long PololuMagneticEncoder::countsEncoderRight = 0; // Inicializa o atributo estático countsEncoderRight

/*
  Configura o número dos pinos utilizados para a leitura dos encoders, o modo de utilização dos pinos como entrada e as funções de interrupção para cada pino.

  Configura os pinos como INPUT_PULLUP.
  Configura as funções attachInterrupt no modo CHANGE.

  @param pin1 Número do pino do ESP32 conectado ao pino OUT A do encoder esquerdo do robô.
  @param pin2 Número do pino do ESP32 conectado ao pino OUT B do encoder esquerdo do robô.
  @param pin3 Número do pino do ESP32 conectado ao pino OUT A do encoder direito do robô.
  @param pin4 Número do pino do ESP32 conectado ao pino OUT B do encoder direito do robô.
*/
void PololuMagneticEncoder::setupEncoders(int pin1, int pin2, int pin3, int pin4)
{
  pin[0] = pin1;
  pin[1] = pin2;
  pin[2] = pin3;
  pin[3] = pin4;

  pinMode(pin[0], INPUT_PULLUP);
  pinMode(pin[1], INPUT_PULLUP);
  pinMode(pin[2], INPUT_PULLUP);
  pinMode(pin[3], INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pin[0]), PololuMagneticEncoder::handleInterruptEncoderLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin[1]), PololuMagneticEncoder::handleInterruptEncoderLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin[2]), PololuMagneticEncoder::handleInterruptEncoderRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin[3]), PololuMagneticEncoder::handleInterruptEncoderRightB, CHANGE);
}


void PololuMagneticEncoder::handleInterruptEncoderLeftA()
{
  if (digitalRead(pin[1]) != digitalRead(pin[0])) 
  {
    countsEncoderLeft ++;
  } 
  else 
  {
    countsEncoderLeft --;
  }
}

void PololuMagneticEncoder::handleInterruptEncoderLeftB()
{
  if (digitalRead(pin[1]) == digitalRead(pin[0])) 
  {
    countsEncoderLeft ++;
  } 
  else 
  {
    countsEncoderLeft --;
  }
}

void PololuMagneticEncoder::handleInterruptEncoderRightA()
{
  if (digitalRead(pin[3]) != digitalRead(pin[2])) 
  {
    countsEncoderRight ++;
  } 
  else 
  {
    countsEncoderRight --;
  }
}

void PololuMagneticEncoder::handleInterruptEncoderRightB()
{
  if (digitalRead(pin[3]) == digitalRead(pin[2])) 
  {
    countsEncoderRight ++;
  } 
  else 
  {
    countsEncoderRight --;
  }
}

long PololuMagneticEncoder::getCountsEncoderLeft()
{
  return countsEncoderLeft;
}

long PololuMagneticEncoder::getCountsAndResetEncoderLeft()
{
  int tmp = countsEncoderLeft;
  countsEncoderLeft = 0;
  
  return tmp;
}

long PololuMagneticEncoder::getCountsEncoderRight()
{
  return countsEncoderRight;
}

long PololuMagneticEncoder::getCountsAndResetEncoderRight()
{
  int tmp = countsEncoderRight;
  countsEncoderRight = 0;
  
  return tmp;
}