clc;
clear;

psi1_arr = [];
psi2_arr = [];

rho1_arr = [];
rho2_arr = [];

x_arr = [];

for x = 0:0.01:2*pi
    psi1 = cos(x);
    psi2 = sin(x);
    
    eta = 10;
    rho1 = min(psi1,psi2);
    rho2 = -1/eta*log(exp(-eta*psi1)+exp(-eta*psi2));
    
    psi1_arr = [psi1_arr; psi1];
    psi2_arr = [psi2_arr; psi2];
    rho1_arr = [rho1_arr; rho1];
    rho2_arr = [rho2_arr; rho2];
    x_arr = [x_arr; x];
    
end

figure(1)
subplot(1,2,1)
plot(x_arr,psi1_arr,'k-', 'LineWidth', 1.5); hold on;
plot(x_arr,psi2_arr,'r-', 'LineWidth', 1.5);
legend('cos(x)', 'sin(x)')
xlim([0, 2*pi])
grid on;
axis square

subplot(1,2,2)
plot(x_arr,rho1_arr,'k-', 'LineWidth', 1.5); hold on;
plot(x_arr,rho2_arr,'r-', 'LineWidth', 1.5);
legend('min', 'smooth approximation')
xlim([0, 2*pi])
grid on;
axis square