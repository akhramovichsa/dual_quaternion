clc; clear all;

syms x1 z2 z3; %  x2 x3;
syms k1 k2 k3;
syms u;

x2 = z2 - k1*x1;
x3 = z3 - x1*(1 + k1*k2) - x2*(k1 + k2);

x1_dot = z2 - k1*x1;
z2_dot = x3 + k1*(z2 - k1*x1);
z3_dot = u + x2*(1 + k1*k2) + x3*(k1 + k2);

dv_dt = x1*x1_dot + z2*z2_dot + z3*z3_dot + k1*x1^2 + k2*z2^2 + k3*z3^2

dv_dt = simplify(dv_dt)

% dv_dt = strcat(char(dv_dt), ' = 0')
% solve(dv_dt, u)

uu = u + 2*z2 - 2*k1*x1 - k2*x1 + k1*z3 + k2*z3 + k3*z3 + k1^3*x1 - k1^2*z2 - k2^2*z2 - k1*k2*z2

uu = subs(uu, z2, 'x2 + k1*x1')
uu = subs(uu, z3, 'x3 + x1*(1 + k1*k2) + x2*(k1 + k2)')

simplify(uu)

u = -x1*(k1 + k3 + k1*k2*k3) - x2*(2 + k1*k2 + k2*k3 + k1*k3) - x3*(k1 + k2)

%% Халил

phi0     = -k1*x1;
dphi0_dt = -k1*z2;

u = k1*x2 + k1*k1*x1 - x1 - k2*x1 + k2*k1*x1;


% % z2 = x2 + k1*x1
% % z3 = x3 + x1*(1 + k1*k2) + x2*(k1 + k2)
% % 
% % eq_1 = z3 - x1*(1 + k1*k2) - x2*(k1 + k2) % = x3
% % eq_2 = x2*(1 + k1*k2) + x3*(k1 + k2)
% % 
% % % dV_dt = x1*(z2 - k1*x1) + x2*eq_1 + z3*eq_2 + k1*(x1^2) + k2*(x2^2) + k3*(z3^2)
% % 
% % dV_dt = x1*x2 + x2*x3 + z3*u + z3*eq_2 + k1*(x1^2) + k2*(x2^2) + k3*(z3^2)
% % 
% % u = -(x1*x2 + x2*x3 + z3*eq_2 + k1*(x1^2) + k2*(x2^2) + k3*(z3^2))/z3
