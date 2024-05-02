clc; close all; clear all

num = [0.0029];
den = [1 -0.3542];
T = 0.1;
G = tf(num,den,0.1)
pidTuner(G)

Kpe = 26.4752; % Roda esquerda
Kie = 529.5046; % Roda esquerda

Kp = 25.3988; % Roda direita
Ki = 510.9769; % Roda direita