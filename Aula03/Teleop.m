close all; clear; clc;
%Porta Serial utilizada
portaSerial = 'COM3';

if(~exist('esp32','var'))
   esp32 = configSerial(portaSerial); 
end

try
    disp('Digite a opção desejada:');
    ch = getkey(1);
    while(ch ~= 27)
        fwrite(esp32,ch);
        ch = getkey;
    end
    fclose(esp32);
    disp('Conexão encerrada');
catch
    fclose(esp32);
    disp('Conexão encerrada');
end

% Erro:Use INSTRFIND to determine if other instrument objects are connected to the requested device. 
% Solução: fclose(instrfindall);