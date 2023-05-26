clc;
clear;

r1 = [0; 0];
r2 = [6; 0];
r3 = [12; 0];
r4 = [3; 3];
r5 = [9; 3];
rx = [r1(1), r2(1), r3(1), r4(1), r5(1)];
ry = [r1(2), r2(2), r3(2), r4(2), r5(2)];

rx_arr = [];
ry_arr = [];

rho_arr = [];
p_arr = [];
t_arr = [];

u4 = [0; 0];
u5 = [0; 0];

eta = 50;
    
dt = 0.1;
for t = 0:dt:5
    ts = 1;
    %% plot agents
    rx_arr = [rx_arr; rx];
    ry_arr = [ry_arr; ry];
    figure(1)
    subplot(1,2,1)
    plot(rx(1:3), ry(1:3), 'ko'); hold on;
    plot(rx(4:5), ry(4:5), 'bo');
    plot(rx_arr, ry_arr, 'k--');
    
    % plot goal
    gx = 5; gy = 1.5;
    g = [gx; gy];
    plot(gx, gy,'og','MarkerSize',40); hold off;
    
    xlim([-1 13]);
    ylim([-0.5 3.5]);
    grid on;
    axis square;
    %% Agent Dynamics
    % follower
    dr1 = (r4-r1);
    dr2 = (r4-r2) + (r5-r2);
    dr3 = (r5-r3);

    % leader
    dr4 = (r1-r4) + (r2-r4) + u4;
    dr5 = (r2-r5) + (r3-r5) + u5;
    
    %% Robustness Semantics
    psi1 = 2-norm(r1,r4);
    psi2 = 2-norm(r2,r4);
    psi3 = 2-norm(r2,r5);
    psi4 = 2-norm(r3,r5);
    
    psi5 = 1-norm(g,r4);
    psi6 = 1-norm(g,r5);
    
    k = 0.5;
    rho1 = -1/eta*log(exp(-eta*psi1)+exp(-eta*psi2)+exp(-eta*psi3)+exp(-eta*psi4))+k;
    rho2 = -1/eta*log(exp(-eta*psi5)+exp(-eta*psi6))+k;
    
    %% Funnel
    l1 = log(19);
    p1 = 19*exp(-l1*t)+1;
    rho1_star = 2;
    t1_star = 1;

    l2 = 0.5*log(9);
    p2 = 9*exp(-l2*(t-1))+1;
    rho2_star = 2;
    t2_star = 3;

    %% Error
    e1 = rho1 - rho1_star;
    e1_bar = e1/p1; % Modulated Error
    eps1 = transform(e1_bar); % Transformed Error

    e2 = rho2 - rho2_star;
    e2_bar = e2/p2; % Modulated Error
    eps2 = transform(e2_bar); % Transformed Error
    
    %% plot robustness
    figure(1)
    subplot(1,2,2)
    if t<=ts
        rho = rho1;
        p = 2-p1;
    else
        rho = rho2;
        p = 2-p2;
    end
    rho_arr = [rho_arr; rho];
    p_arr = [p_arr; p];
    t_arr = [t_arr; t];
    
    plot(t_arr,rho_arr,'r-','LineWidth', 1.5); hold on;
    plot(t_arr,p_arr,'k-','LineWidth', 1.5);
    xlim([0 5]);
    ylim([-18 2]);
    grid on;
    axis square
    
    %% Control
    x1_1 = r1(1);
    x1_2 = r2(1);
    x1_3 = r3(1);
    x1_4 = r4(1);
    x1_5 = r5(1);
    
    x2_1 = r1(2);
    x2_2 = r2(2);
    x2_3 = r3(2);
    x2_4 = r4(2);
    x2_5 = r5(2);
    
    % Control 1
    if t<=ts
        u4x = -eps1*((eta*exp(eta*(((x1_1 - x1_4)^2 + (x2_1 - x2_4)^2)^(1/2) - 2))*(2*x1_1 - 2*x1_4))/(2*((x1_1 - x1_4)^2 + (x2_1 - x2_4)^2)^(1/2)) + (eta*exp(eta*(((x1_2 - x1_4)^2 + (x2_2 - x2_4)^2)^(1/2) - 2))*(2*x1_2 - 2*x1_4))/(2*((x1_2 - x1_4)^2 + (x2_2 - x2_4)^2)^(1/2)))/(eta*(exp(eta*(((x1_1 - x1_4)^2 + (x2_1 - x2_4)^2)^(1/2) - 2)) + exp(eta*(((x1_2 - x1_4)^2 + (x2_2 - x2_4)^2)^(1/2) - 2)) + exp(eta*(((x1_2 - x1_5)^2 + (x2_2 - x2_5)^2)^(1/2) - 2)) + exp(eta*(((x1_3 - x1_5)^2 + (x2_3 - x2_5)^2)^(1/2) - 2))));
        u4y = -eps1*((eta*exp(eta*(((x1_1 - x1_4)^2 + (x2_1 - x2_4)^2)^(1/2) - 2))*(2*x2_1 - 2*x2_4))/(2*((x1_1 - x1_4)^2 + (x2_1 - x2_4)^2)^(1/2)) + (eta*exp(eta*(((x1_2 - x1_4)^2 + (x2_2 - x2_4)^2)^(1/2) - 2))*(2*x2_2 - 2*x2_4))/(2*((x1_2 - x1_4)^2 + (x2_2 - x2_4)^2)^(1/2)))/(eta*(exp(eta*(((x1_1 - x1_4)^2 + (x2_1 - x2_4)^2)^(1/2) - 2)) + exp(eta*(((x1_2 - x1_4)^2 + (x2_2 - x2_4)^2)^(1/2) - 2)) + exp(eta*(((x1_2 - x1_5)^2 + (x2_2 - x2_5)^2)^(1/2) - 2)) + exp(eta*(((x1_3 - x1_5)^2 + (x2_3 - x2_5)^2)^(1/2) - 2))));
 
        u5x = -eps1*((eta*exp(eta*(((x1_2 - x1_5)^2 + (x2_2 - x2_5)^2)^(1/2) - 2))*(2*x1_2 - 2*x1_5))/(2*((x1_2 - x1_5)^2 + (x2_2 - x2_5)^2)^(1/2)) + (eta*exp(eta*(((x1_3 - x1_5)^2 + (x2_3 - x2_5)^2)^(1/2) - 2))*(2*x1_3 - 2*x1_5))/(2*((x1_3 - x1_5)^2 + (x2_3 - x2_5)^2)^(1/2)))/(eta*(exp(eta*(((x1_1 - x1_4)^2 + (x2_1 - x2_4)^2)^(1/2) - 2)) + exp(eta*(((x1_2 - x1_4)^2 + (x2_2 - x2_4)^2)^(1/2) - 2)) + exp(eta*(((x1_2 - x1_5)^2 + (x2_2 - x2_5)^2)^(1/2) - 2)) + exp(eta*(((x1_3 - x1_5)^2 + (x2_3 - x2_5)^2)^(1/2) - 2))));
        u5y = -eps1*((eta*exp(eta*(((x1_2 - x1_5)^2 + (x2_2 - x2_5)^2)^(1/2) - 2))*(2*x2_2 - 2*x2_5))/(2*((x1_2 - x1_5)^2 + (x2_2 - x2_5)^2)^(1/2)) + (eta*exp(eta*(((x1_3 - x1_5)^2 + (x2_3 - x2_5)^2)^(1/2) - 2))*(2*x2_3 - 2*x2_5))/(2*((x1_3 - x1_5)^2 + (x2_3 - x2_5)^2)^(1/2)))/(eta*(exp(eta*(((x1_1 - x1_4)^2 + (x2_1 - x2_4)^2)^(1/2) - 2)) + exp(eta*(((x1_2 - x1_4)^2 + (x2_2 - x2_4)^2)^(1/2) - 2)) + exp(eta*(((x1_2 - x1_5)^2 + (x2_2 - x2_5)^2)^(1/2) - 2)) + exp(eta*(((x1_3 - x1_5)^2 + (x2_3 - x2_5)^2)^(1/2) - 2))));
    end

    %Control 2
    if t>ts
        u4x = -eps2*(exp(eta*(((gx - x1_4)^2 + (gy - x2_4)^2)^(1/2) - 1))*(2*gx - 2*x1_4))/(2*((gx - x1_4)^2 + (gy - x2_4)^2)^(1/2)*(exp(eta*(((gx - x1_4)^2 + (gy - x2_4)^2)^(1/2) - 1)) + exp(eta*(((gx - x1_5)^2 + (gy - x2_5)^2)^(1/2) - 1))));
        u4y = -eps2*(exp(eta*(((gx - x1_4)^2 + (gy - x2_4)^2)^(1/2) - 1))*(2*gy - 2*x2_4))/(2*((gx - x1_4)^2 + (gy - x2_4)^2)^(1/2)*(exp(eta*(((gx - x1_4)^2 + (gy - x2_4)^2)^(1/2) - 1)) + exp(eta*(((gx - x1_5)^2 + (gy - x2_5)^2)^(1/2) - 1))));

        u5x = -eps2*(exp(eta*(((gx - x1_5)^2 + (gy - x2_5)^2)^(1/2) - 1))*(2*gx - 2*x1_5))/(2*((gx - x1_5)^2 + (gy - x2_5)^2)^(1/2)*(exp(eta*(((gx - x1_4)^2 + (gy - x2_4)^2)^(1/2) - 1)) + exp(eta*(((gx - x1_5)^2 + (gy - x2_5)^2)^(1/2) - 1))));
        u5y = -eps2*(exp(eta*(((gx - x1_5)^2 + (gy - x2_5)^2)^(1/2) - 1))*(2*gy - 2*x2_5))/(2*((gx - x1_5)^2 + (gy - x2_5)^2)^(1/2)*(exp(eta*(((gx - x1_4)^2 + (gy - x2_4)^2)^(1/2) - 1)) + exp(eta*(((gx - x1_5)^2 + (gy - x2_5)^2)^(1/2) - 1))));
    end
    u4 = [u4x; u4y];
    u5 = [u5x; u5y];
    
    %% update
    r1 = r1 + dr1*dt;
    r2 = r2 + dr2*dt;
    r3 = r3 + dr3*dt;
    r4 = r4 + dr4*dt;
    r5 = r5 + dr5*dt; 
    rx = [r1(1), r2(1), r3(1), r4(1), r5(1)];
    ry = [r1(2), r2(2), r3(2), r4(2), r5(2)];
    
    pause(0.1)
end
    
%% 2-Norm
function f = norm(a,b) 
    x = a(1)-b(1);
    y = a(2)-b(2);
    f = sqrt(x^2+y^2);
end

%% Transformation Function
function te = transform(e) 
    te = log(-(1+e)/(e));
end

%% Goal Circle
function h = drawCircle(x,y,r,MarkerFaceColor,MarkerEdgeColor)
hold on
c = [x y];
pos = [c-r 2*r 2*r];
r = rectangle('Position',pos,'Curvature',[1 1], ...
    'FaceColor', MarkerFaceColor, 'Edgecolor',MarkerEdgeColor);
hold off
end