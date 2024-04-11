/*
  PololuMagneticEncoder.h - Library for using Pololu Magnetic Encoders
  Author: Ana Sophia Cavalcanti Alves Vilas Boas
  Creation date: April 2023
*/

#ifndef POLOLUENCODER_H
#define POLOLUENCODER_H

class PololuMagneticEncoder
{
public:
  static int pin[4]; // Armazena os pinos do ESP32 conectados aos pinos OUT A e OUT B do encoder esquerdo e direito do robô, respectivamente.
  static volatile long countsEncoderLeft; // Armazena a contagem de pulsos lidos nos pinos OUT A e OUT B do encoder esquerdo do robô.
  static volatile long countsEncoderRight; // Armazena a contagem de pulsos lidos nos pinos OUT A e OUT B do encoder direito do robô.
  void setupEncoders(int, int, int, int); // Configura o número dos pinos utilizados para a leitura dos encoders, o modo de utilização dos pinos como entrada e as funções de interrupção para cada pino.
  static void handleInterruptEncoderLeftA(); // Faz a contagem de pulsos do pino OUT A do encoder esquerdo caso o sinal de entrada sofra mudança de estado.
  static void handleInterruptEncoderLeftB(); // Faz a contagem de pulsos do pino OUT B do encoder esquerdo caso o sinal de entrada sofra mudança de estado.
  static void handleInterruptEncoderRightA(); // Faz a contagem de pulsos do pino OUT A do encoder direito caso o sinal de entrada sofra mudança de estado.
  static void handleInterruptEncoderRightB(); // Faz a contagem de pulsos do pino OUT B do encoder direito caso o sinal de entrada sofra mudança de estado.
	long getCountsEncoderLeft(); // Retorna o número de pulsos contados no encoder esquerdo.
	long getCountsEncoderRight(); // Retorna o número de pulsos contados no encoder direito.
  long getCountsAndResetEncoderLeft(); // Retorna o número de pulsos contados no encoder esquerdo e zera a contagem.
	long getCountsAndResetEncoderRight();	// Retorna o número de pulsos contados no encoder direito e zera a contagem.
};

# endif