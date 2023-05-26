clc;
clear;

syms x [2 5] real
syms eta ;

r1 = x(:,1);
r2 = x(:,2);
r3 = x(:,3);
r4 = x(:,4);
r5 = x(:,5);
%% Robustness Semantics
% psi1 = 2-norm(r1,r4);
% psi2 = 2-norm(r2,r4);
% 
% psi3 = 2-norm(r2,r5);
% psi4 = 2-norm(r3,r5);
% 
% rho1 = -1/eta*log(exp(-eta*psi1)+exp(-eta*psi2));
% rho2 = -1/eta*log(exp(-eta*psi3)+exp(-eta*psi4));

%% Robustness Semantics 2
psi1 = 2-norm(r1,r4);
psi2 = 2-norm(r2,r4);
psi3 = 2-norm(r2,r5);
psi4 = 2-norm(r3,r5);

syms gx gy
g = [gx; gy];
psi5 = 1-norm(g,r4);
psi6 = 1-norm(g,r5);

rho1 = -1/eta*log(exp(-eta*psi1)+exp(-eta*psi2)+exp(-eta*psi3)+exp(-eta*psi4));
rho2 = -1/eta*log(exp(-eta*psi5)+exp(-eta*psi6));

%% d_rho/d_xL
diff(rho1,x1_4)
diff(rho1,x2_4)
diff(rho1,x1_5)
diff(rho1,x2_5)

diff(rho2,x1_4)
diff(rho2,x2_4)
diff(rho2,x1_5)
diff(rho2,x2_5)

%% 2-Norm
function f = norm(a,b) 
    x = a(1)-b(1);
    y = a(2)-b(2);
    f = sqrt(x^2+y^2);
end