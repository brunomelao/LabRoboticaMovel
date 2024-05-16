clc, clear, close all

data = load('t100a05');
pv = 900;
D = 0.042;
dT = 0.1;
dS = ((pi*D)/pv)/dT;

y = dS*data.d(:,2);
u = data.d(:,1);
N = length(y);

Y = y(2:N);
X = [y(1:N-1) u(1:N-1)];

theta = inv(X'*X)*X'*Y

yh = [0];
for i = 2:N
    yh(i) = theta(1) *  y(i - 1) + theta(2) * u(i-1); 
    
end

for i = 1:N
    MSE=sum((yh(i)-y(i))^2)/N;
end

% ------------ Plots -----------
figure()
subplot(1,1,1)
plot(data.dT, y, 'b', 'linewidth', 2)
hold on
plot(data.dT, yh, 'r','linewidth', 2)
hold on
grid on
title('Identificação do Sistema')
xlabel('Tempo [s]')
ylabel('Velocidade [m/s]')
legend('Medido','Calculado') 

% ----------- Validação ---------
data_t = load('t100a06');
y_t = dS*data_t.d(:,2);
u_t = data_t.d(:,1);

yh_t = [0];
for i = 2:N
    yh_t(i) = theta(1) *  y_t(i - 1) + theta(2) * u_t(i-1);   
    
end

for i = 1:N
    MSE_t=sum((yh_t(i)-y_t(i))^2)/N;
end