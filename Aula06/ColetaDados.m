clc, clear , close all

portaSerial = 'COM3';
d = zeros(250,3);
if(~exist('esp32','var'))
    esp32 = configSerial(portaSerial);
 try
     nA = 50;
     r = 5;
     n = randi(255,[1,50]);
     for j = 1:length(n)
        for i= 1:r
            u(5*(j-1)+i) = n(j);
        end
     end
     
     for i=1:length(u)
         fwrite(esp32, u(i))
%          disp(u(i));
         d(i,1) = fscanf(esp32,'%u');
         disp(d(i,1));
         d(i,2) = fscanf(esp32,'%u');
         d(i,3) = fscanf(esp32,'%u');
         dT(i) = 0.1*i;
     end 
     fwrite(esp32, 0)
     dT = dT';
     save('t100a01','dT', 'd');
     
     fclose(esp32);
  catch
     fclose(esp32);
 end
     
end
