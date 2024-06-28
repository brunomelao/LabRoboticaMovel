mclc, clear, close all

% fclose(instrfind);

% ---------------------- Inicialização de Variaveis --------------------
v = 0;
w = 0;
l = 0.085/2; % Distancia entre rodas 
r = 0.042/2;
vel = [];
tempo = 0;
contador = 0;


portaSerial = 'COM3';
if(~exist('esp32', 'var'))
    esp32 = configSerial(portaSerial);
    pause(2);
    disp('Digite a opcao desejada: ');
    ch = getkey();
    while(ch ~=  27)
        if ch == 30                                                     % up arrow
            v = 0.5;
            w = 0;

        elseif ch == 31                                                 % down arrow
            v = -0.5;
            w =  0;
        elseif ch == 29                                                 % right arrow
            v = 0;
            w = -2;

        elseif ch == 28                                                 % right left
            v = 0;
            w = 2;
        elseif ch == 32                                                 % space
            v = 0;
            w = 0;
        end


        % ----------- Cálculo de velocidade das rodas ----------------
        vD = v + w*l;
        vE = v - w*l;
        
        vD = single(vD);
        vE = single(vE);
        
        vDarray = typecast(vD,'uint8');
        vEarray = typecast(vE,'uint8');

        msg = [1 vEarray vDarray];

        for i=1:length(msg)
            fwrite(esp32, msg(i));
        end

        %vel(contador,1) = 0.0015*fscanf(esp32, '%u');
        %vel(contador,2) = 0.0015*fscanf(esp32, '%u');              


% -------------------------------------------------------------------------
        ch = getkey();
   end
   fclose(esp32);
    
end

% subplot(2,1,1)
% plot(vel(:,1))
% title('Velocidade da roda esquerda')  
% subplot(2,1,2)
% plot(vel(:,2))
% title('Velocidade da roda direita')  
% suptitle('Controle das Rodas')