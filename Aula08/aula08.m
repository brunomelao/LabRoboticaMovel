clc; close all; clear all

num = [0.0032];
den = [1 -0.3112];
T = 0.1;
G = tf(num,den,0.1)
pidTuner(G)

Kp = 24.4377;
Ki = 488.7538;