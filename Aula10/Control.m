clc, clear variables, close all;
 %fclose(instrfind);

portaSerial = ('COM3');
if(~exist('esp32','var'))
    esp32 = configSerial(portaSerial);
    vD = 0.5;
    vE = 0.2;
    tempo = 0;
    contador = 0;
    vel = [];
    pause(2);
    
    vD = single(vD);
    vE = single(vE);
    
    vDarray = typecast(vD,'uint8');
    vEarray = typecast(vE,'uint8');
    
    msg = [1 vEarray vDarray];
    
    tic ;
    while tempo < 8
        dt = toc;
            if(dt > 0.2)
                tempo = tempo + dt;
                tic;
                contador = contador + 1; % será utilizado na próxima prática
                for i=1:length(msg)
                    fwrite(esp32, msg(i));
                end
                 vel(contador,1) = 0.0015*fscanf(esp32, '%u');
                 vel(contador,2) = 0.0015*fscanf(esp32, '%u');              
            end
            pause(0.01);
    end
    
    msg = [1 typecast(0,'uint8') typecast(0,'uint8')];


        for i=1:length(msg)
            fwrite(esp32, msg(i));
        end

fclose(esp32);
end

subplot(2,1,1)
plot(vel(:,1))
title('Velocidade da roda esquerda')  
subplot(2,1,2)
plot(vel(:,2))
title('Velocidade da roda direita')  