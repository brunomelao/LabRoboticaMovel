clc; close all; clear all

num = [0.0028];
den = [1 -0.3497];
T = 0.1;
G = tf(num,den,0.1)
pidTuner(G)
 
Kpe = 25.2573; % Roda esquerda  horario 
Kie = 505.146; % Roda esquerda  horario

Kpe = 26.4752; % Roda esquerda  anti-horario 
Kie = 529.5046; % Roda esquerda  anti-horario

Kp = 25.3988; % Roda direita
Ki = 510.9769; % Roda direita

Kp = 25.7384; % Roda direita horario
Ki = 514.7678; % Roda direita horario