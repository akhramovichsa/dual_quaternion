function syms_equation_2
clc; clear all;

% step_another();
% return;

step_2_with_z_theta();
return;





syms r_y V_x V_y z_theta theta omega_z k_1 k_2;

% z_theta = theta   + (V_y + k_1*r_y)/V_x;
theta_d = z_theta - (V_y + k_1*r_y)/V_x;

aaa = r_y*(V_x*theta_d + V_y) + z_theta*(omega_z + (k_1/V_x)*(V_x*theta_d + V_y)) + k_1*r_y^2 + k_2*z_theta^2
expand(aaa)
simplify(aaa)

bbb = (V_x*omega_z + V_x^2*r_y - k_1^2*r_y + V_x*k_1*(theta   + (V_y + k_1*r_y)/V_x) + V_x*k_2*(theta   + (V_y + k_1*r_y)/V_x))/V_x
expand(bbb)
simplify(bbb)


omega_z + V_x*r_y + k_1*theta + k_2*theta + (V_y*k_1)/V_x + (V_y*k_2)/V_x + (k_1*k_2*r_y)/V_x
% -omega_z = r_y*(V_x + (k_1*k_2)/V_x) + theta*(V_y/V_x)*(k_1 + k_2) 


disp('STEP 3 -----------------------------------------------------------');
clc; clear all;
syms Vx Vy ry z_theta theta z_omega_z omega_z MAz Jzz ly FPx;
syms k1 k2 k3;

theta_d   = z_theta - (Vy + k1*ry)/Vx;
omega_z_d = z_omega_z - (1/Vx) * (ry*(1 + k1*k2) + Vy*theta*(k1 + k2));

ry_dot        = Vx*theta_d + Vy;
z_theta_dot   = omega_z_d;
z_omega_z_dot = MAz/Jzz -(ly/Jzz) * FPx;
 
V3_dot = ry*ry_dot + z_theta*z_theta_dot + z_omega_z*z_omega_z_dot;
 
% W3 = k1*ry^2 + k2*theta^2 +k3*z_omega_z^2;
% 
% V3_dot_W3 = V3_dot - W3;
% % aaa = (MAz/Jzz - (FPx*ly)/Jzz) - k1*ry^2 - k2*theta^2 - k3*z_omega_z^2 + ry*(Vy + Vx*theta) + theta*(z_omega_z - (ry*(k1*k2 + 1) + Vy*theta*(k1 + k2))/Vx
% subs(V3_dot_W3, 'z_omega_z', '(omega_z - (1/Vx) * (ry*(1 + k1*k2) + Vy*theta*(k1 + k2)))' ) 
% expand(V3_dot_W3)
% solve(V3_dot_W3, FPx);



disp('STEP 3a -----------------------------------------------------------');
% clc; clear all;
% syms Vx Vy ry theta z_omega_z omega_z MAz Jzz ly FPx;
% syms k1 k2 k3;
% 
% ry_dot        = Vx*theta + Vy;
% theta_dot     = z_omega_z - (1/Vx) * (ry*(1 + k1*k2) + Vy*theta*(k1 + k2));
% z_omega_z_dot = MAz/Jzz -(ly/Jzz) * FPx;
% 
% V3_dot = ry*ry_dot + theta*theta_dot + z_omega_z*z_omega_z_dot;
% 
% W3 = k1*ry^2 + k2*theta^2 +k3*z_omega_z^2;
% 
% V3_dot_W3 = V3_dot + W3;
% % aaa = (MAz/Jzz - (FPx*ly)/Jzz) - k1*ry^2 - k2*theta^2 - k3*z_omega_z^2 + ry*(Vy + Vx*theta) + theta*(z_omega_z - (ry*(k1*k2 + 1) + Vy*theta*(k1 + k2))/Vx
% subs(V3_dot_W3, 'z_omega_z', '(omega_z - (1/Vx) * (ry*(1 + k1*k2) + Vy*theta*(k1 + k2)))' ) 
% expand(V3_dot_W3)
% solve(V3_dot_W3, FPx);


% z_theta = theta   + (V_y + k_1*r_y)/V_x;
% bbb = - r_y*k_1^2/V_x + z_theta*k_1 + omega_z + V_x*r_y + k_2*z_theta
% simplify(bbb)

% aaa = r_y*(V_x*(z_theta-(V_y + k_1*r_y)/V_x )+V_y) + z_theta*(omega_z + k_1*(V_x*(z_theta - (V_y+k_1*r_y)/V_x ) + V_y)) + k_1*r_y^2 + k_2*z_theta^2
% simplify(aaa)
% 
% -r_y*k_1^2 + V_x*z_theta*k_1 + omega_z + V_x*r_y + k_2*z_theta
% 
% bbb = -r_y*k_1^2 + V_x*(theta + (V_y+k_1*r_y)/V_x)*k_1 + omega_z + V_x*r_y + k_2*theta + (V_y+k_1*r_y)/V_x
% simplify(bbb)
% 
% (V_y + V_x*omega_z + k_1*r_y + V_x^2*r_y + V_x*V_y*k_1 + V_x*k_2*theta + V_x^2*k_1*theta)/V_x
% 
% V_y/V_x + omega_z + k_1*r_y/V_x + V_x*r_y + V_y*k_1 + k_2*theta + V_x*k_1*theta

% syms H theta k1 k2 Vy z2 u;
% 
% phi     = asin(-k1*H/Vy);
% phi_dot = - ( 1/sqrt(1 - (k1*H/Vy)^2) ) * (k1/Vy) * (Vy*sin(theta));
% 
% VL_dot = H*Vy*sin(z2 + phi) + z2*(u - phi_dot) + k1*H^2 + k2*z2^2
% 
% aaa = 'H^2*k1 + k2*z2^2 + z2*(u + (k1*sin(theta))/(1 - (H^2*k1^2)/Vy^2)^(1/2)) + H*Vy*sin(z2 - asin((H*k1)/Vy))'
% u_omega_z = strrep(aaa, 'z2', 'theta + asin(-k1*H/Vy)')
% 
% bbb = eval(u_omega_z)
% 
% simplify(bbb)

syms q0 qx qy q_z;
syms Vx Vy Vz;

q      = [q0; qx; qy; q_z];
q_conj = [q0; -qx; -qy; -q_z];
V      = [0; Vx; Vy; Vz]; 

qq = quat_mul(q, V)
quat_mul(qq, q_conj)

solve('q0*(Vy*q0 + Vx*q_z - Vz*qx) + q_z*(Vx*q0 - Vy*q_z + Vz*qy) - qx*(Vz*q0 - Vx*qy + Vy*qx) + qy*(Vz*q_z + Vx*qx + Vy*qy)', q_z)
end

% --------------------------------------------------
function step_2_with_z_theta()
syms r_y V_x V_y z_theta z_omega_z omega_z;
syms k_1 k_2 k_3;
syms Maz J_zz l_y F_px;

theta_d = - (V_y +k_1*r_y)/V_x;
theta   = z_theta + theta_d;

r_y_dot     = expand(V_x*theta + V_y)
z_theta_dot = expand(omega_z + (k_1/V_x)*r_y_dot)

dVL2 = r_y*r_y_dot + z_theta*z_theta_dot == - k_1*r_y^2 - k_2*z_theta^2

omega_z_d = expand(solve(dVL2, omega_z))
phi_2     = omega_z_d

phi_2_dot = diff(phi_2, r_y)*r_y_dot + diff(phi_2, z_theta)*z_theta_dot


% Третий шаг

r_y_dot       = expand(V_x*theta + V_y)
z_theta_dot   = expand((z_omega_z + omega_z_d) + (k_1/V_x)*r_y_dot)

phi_2_dot = diff(phi_2, r_y)*r_y_dot + diff(phi_2, z_theta)*z_theta_dot

z_omega_z_dot = Maz/J_zz - l_y/J_zz*F_px - phi_2_dot

% z_omega_z_dot = subs(z_omega_z_dot, omega_z, '(z_omega_z - (- V_x*r_y - k_1*z_theta - k_2*z_theta + (k_1^2*r_y)/V_x ) )')

% aaaaaaaa = simplify(z_omega_z_dot)


dVL3 = r_y*r_y_dot + z_theta*z_theta_dot + z_omega_z*z_omega_z_dot + k_1*r_y^2 + k_2*z_theta^2 + k_3*z_omega_z^2
dVL3 = dVL3/z_omega_z

%F_px_d = solve(dVL3, F_px)

%F_px_d = simplify(F_px_d)
dVL3 = simplify(expand(dVL3))
dVL3_f = collect(dVL3, [z_omega_z, z_theta, r_y])

clear theta omega_z;
syms theta omega_z;
dVL3_f1 = subs(dVL3_f, z_omega_z, (omega_z - (-V_x*r_y - k_1*z_theta - k_2*z_theta + (k_1^2*r_y)/V_x) )  )

dVL3_f1 = subs(dVL3_f1, z_theta, (theta + (V_y + k_1*r_y)/V_x)  )
  
dVL3_f1 = expand(dVL3_f1)
  
dVL3_f1 = collect(dVL3_f1, [omega_z, theta, r_y])

F_px_d = solve(dVL3_f1, F_px)

return;



% Коненое выражение регулятора
F_px_d = subs(F_px_d, z_omega_z, '(omega_z - (-V_x*r_y - k_1*z_theta - k_2*z_theta + (k_1^2*r_y)/V_x) ) ')
F_px_d = subs(F_px_d, z_theta,   '(theta + (V_y + k_1*r_y)/V_x)')

aaa = simplify(expand(F_px_d))
collect((Maz*V_x + J_zz*V_x*((V_y + k_1*r_y)/V_x + theta) + J_zz*V_x^3*((V_y + k_1*r_y)/V_x + theta) - J_zz*k_1^3*r_y + J_zz*V_x^2*k_2*r_y - 2*J_zz*k_1^2*k_2*r_y + J_zz*V_x*k_1*(omega_z + V_x*r_y + k_1*((V_y + k_1*r_y)/V_x + theta) + k_2*((V_y + k_1*r_y)/V_x + theta) - (k_1^2*r_y)/V_x) + J_zz*V_x*k_2*(omega_z + V_x*r_y + k_1*((V_y + k_1*r_y)/V_x + theta) + k_2*((V_y + k_1*r_y)/V_x + theta) - (k_1^2*r_y)/V_x) + J_zz*V_x*k_3*(omega_z + V_x*r_y + k_1*((V_y + k_1*r_y)/V_x + theta) + k_2*((V_y + k_1*r_y)/V_x + theta) - (k_1^2*r_y)/V_x) + J_zz*V_x*k_1^2*((V_y + k_1*r_y)/V_x + theta) + J_zz*V_x*k_2^2*((V_y + k_1*r_y)/V_x + theta) + 3*J_zz*V_x*k_1*k_2*((V_y + k_1*r_y)/V_x + theta))/(V_x*l_y), omega_z)
return;
 
% omega_z_d = solve('(k_1^2*r_y*z_theta)/V_x + 2*k_1*r_y^2 + k_1*z_theta^2 + (2*V_y*k_1*z_theta)/V_x + V_x*r_y*z_theta + 2*V_y*r_y + k_2*z_theta^2 + omega_z*z_theta = 0', omega_z)


% aa = simplify(expand(omega_z_d))
% bb = subs(aa, 'z_theta', 'theta + (V_y + k_1*r_y)/V_x')
% expand(bb)
 
% A = V_x*(z_theta - (V_y +k_1*r_y)/V_x) + V_y;
% eq = r_y*A + z_theta*(omega_z + (k_1/V_x) * A) + k_1*r_y^2 + k_2*z_theta^2
% sol_eq = solve('r_y*(V_y + V_x*(z_theta - (V_y + k_1*r_y)/V_x)) + z_theta*(omega_z + (k_1*(V_y + V_x*(z_theta - (V_y + k_1*r_y)/V_x)))/V_x) + k_1*r_y^2 + k_2*z_theta^2 = 0', omega_z)
% simplify(sol_eq)
% 
% aa = -(r_y*V_x^2 + z_theta*V_x*k_1 + k_2*z_theta*V_x - r_y*k_1^2)/V_x
% bb = subs(aa, 'z_theta', 'theta + (V_y + k_1*r_y)/V_x')
% expand(bb)
end


function step_2()
syms r_y V_x V_y z_theta theta omega_z k_1 k_2;

A = V_x*(z_theta - (V_y +k_1*r_y)/V_x) + V_y;
eq = r_y*A + z_theta*(omega_z + (k_1/V_x) * A) + k_1*r_y^2 + k_2*z_theta^2
sol_eq = solve('r_y*(V_y + V_x*(z_theta - (V_y + k_1*r_y)/V_x)) + z_theta*(omega_z + (k_1*(V_y + V_x*(z_theta - (V_y + k_1*r_y)/V_x)))/V_x) + k_1*r_y^2 + k_2*z_theta^2 = 0', omega_z)
simplify(sol_eq)

aa = -(r_y*V_x^2 + z_theta*V_x*k_1 + k_2*z_theta*V_x - r_y*k_1^2)/V_x
bb = subs(aa, 'z_theta', 'theta + (V_y + k_1*r_y)/V_x')
expand(bb)
end
function step_another()
syms r_y V_x V_y z_theta z_omega_z omega_z;
syms k_1 k_2 k_3;
syms Maz J_zz l_y F_px;

aaa = r_y*(2*V_y + V_x*z_theta + k_1*r_y)

end





function q_out = quat_mul(q, p)
q_out = [q(1)*p(1) - q(2:4)'*p(2:4); q(1)*p(2:4) + p(1)*q(2:4) + cross(q(2:4), p(2:4))];
end

% q0*(Vy*q0 + Vx*q_z - Vz*qx) + q_z*(Vx*q0 - Vy*q_z + Vz*qy) - qx*(Vz*q0 - Vx*qy + Vy*qx) + qy*(Vz*q_z + Vx*qx + Vy*qy)

