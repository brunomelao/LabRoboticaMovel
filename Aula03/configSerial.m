%Função
function obj = configSerial(portaSerial)
    obj = serial(portaSerial);
    set(obj,'DataBits',8);
    set(obj,'StopBits',1);
    set(obj,'BaudRate',115200);
    set(obj,'Parity','none');
    fopen(obj);   
end