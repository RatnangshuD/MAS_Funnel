clc;
clear;

r1 = [0; 0];
r2 = [6; 0];
r3 = [12; 0];
r4 = [3; 3];
r5 = [9; 3];
rx = [r1(1), r2(1), r3(1), r4(1), r5(1)];
ry = [r1(2), r2(2), r3(2), r4(2), r5(2)];

u4 = [0; 0];
u5 = [0; 0];

dt = 0.1;

syms x [2 5] real

%% Multi-Agent Dynamics
% follower
dx1 = (x(:,4) - x(:,1));
dx2 = (x(:,4) - x(:,2)) + (x(:,5) - x(:,2));
dx3 = (x(:,5) - x(:,3));

% leader
dx4 = (x(:,1) - x(:,4)) + (x(:,2) - x(:,4)) + u4;
dx5 = (x(:,2) - x(:,5)) + (x(:,3) - x(:,5)) + u5;

%% Robustness Semantics
syms eta ;
% phi1
psi1 = 2-norm(x(:,1), x(:,4));
psi2 = 2-norm(x(:,2), x(:,4));
rho1 = -1/eta*log(exp(-eta*psi1)+exp(-eta*psi2));

% phi2
psi3 = 2-norm(x(:,2), x(:,5));
psi4 = 2-norm(x(:,3), x(:,5));
rho2 = -1/eta*log(exp(-eta*psi3)+exp(-eta*psi4));

%% Funnel
syms t
% phi1 0 to t1_star = 1
l1 = log(19);
p1 = 19*exp(-l1*t)+1;
rho1_star = 2;
t1_star = 1;

% phi2 t1_star = 1 to t2_star = 3
l2 = 0.5*log(9);
p2 = 9*exp(-l2*(t-1))+1;
rho2_star = 2;
t2_star = 3;

%% Error
e1 = rho1 - rho1_star;
e1_bar = e1/p1;
eps1 = transform(e1_bar);

e2 = rho2 - rho2_star;
e2_bar = e2/p2;
eps2 = transform(e2_bar);

%%

for t = 0:dt:5
    ts = 1;
    %% plot agents
    figure(1)
    subplot(1,2,1)
    plot(rx, ry, 'ko');
    xlim([-1 13]);
    ylim([-0.5 3.5]);
    grid on;
    axis square;
   
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
    plot(t,rho,'r.'); hold on;
    plot(t,p,'k.');
    xlim([0 5]);
    ylim([-18 2]);
    grid on;
    axis square
    
    %% Control
    ts = t1_star;
    % Control 1
    if t<=ts
        u4x = -eps1*diff(rho1,x1_4);
        u4y = -eps1*diff(rho1,x2_4);

        u5x = -eps1*diff(rho1,x1_5);
        u5y = -eps1*diff(rho1,x2_5);
    end

    %Control 2
    if t>ts
        u4x = -eps1*diff(rho2,x1_4);
        u4y = -eps1*diff(rho2,x2_4);

        u5x = -eps1*diff(rho2,x1_5);
        u5y = -eps1*diff(rho2,x2_5);
    end

    u4 = [u4x; u4y];
    u5 = [u5x; u5y];
    
    %% update
    x1_1 = r1 + dx1*dt;
    r2 = r2 + dx2*dt;
    r3 = r3 + dx3*dt;
    r4 = r4 + dx4*dt;
    r5 = r5 + dx5*dt; 
    rx = [r1(1), r2(1), r3(1), r4(1), r5(1)];
    ry = [r1(2), r2(2), r3(2), r4(2), r5(2)];
    subs(-eps1*diff(rho1,x1_4))
    pause(0.1)
end
    
% 2-Norm
function f = norm(a,b) 
    x = a(1)-b(1);
    y = a(2)-b(2);
    f = sqrt(x^2+y^2);
end

% Transformation Function
function te = transform(e) 
    te = log(-(1+e)/(e));
end

