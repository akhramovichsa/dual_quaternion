
function script_landing()
%           /|\ P = thrust
%            |
%            |
%            |
%  +====================+
%  |         |          |
%  |         o          |
%  |         |          |
%  +====================+
%    /       |         \
%   /        |          \
% ===        |          ===
%            |
%            |
%           \|/ Gravity = mass*g
%
% |OY
% |
% o---- OX

% Начальные условия:
% Высота 100 м, действует сила тяжести и сила тяги двигателя = 0.9 силы
% тяжести, т.е. ЛА должен снижаться
clc; clear all;

% syms_equation();

dual_omega = [0 deg2rad(0) deg2rad(0) deg2rad(0)  0 0 0 0];
dual_q     = dq_from_euler_translation(deg2rad([0 0 0]), [0 100 0]);


options = odeset('RelTol',2e-10);
[t, x] = ode45(@dx_dt, [0:0.1:1], [dual_omega, dual_q], options);

[x]'

dq = x(:, 9:16);
for i = 1:1:length(dq)
    [r1 r2 r3] = dq_get_rotation_euler(dq(i, :));
    [rad2deg([r1 r2 r3]), dq_get_translation_vector(dq(i, :))]
end

% figure(1);
% subplot(2,1,1);
% hold on
% plot(sol_bvp.x, sol_bvp.y(1,:), '-k', 'LineWidth', 1);
% plot(sol_bvp.x, sol_bvp.y(2,:), '--k', 'LineWidth', 1);
% plot(sol_bvp.x, sol_bvp.y(3,:), '-.k', 'LineWidth', 1);
% xlabel('time(sec)')
% ylabel('angular velocity(deg/sec)')
% hold off
% legend('\omega_x','\omega_y','\omega_z');
% grid on;

end

function dx_dt = dx_dt(t, x)
dual_omega = x(1:8)';
dual_q     = x(9:16)';

% dual_omega = [0 0 0 0  0 0 0 0];
% dual_q     = x(1:8)';

m = 10;
g = 9.8;

Fg_nsk = [0 -m*g 0];
Fg = dq_transform_vector(Fg_nsk, dual_q);

dual_Fg = [0 0 0 0  0 Fg];

dual_Fp = [0 0 0 0  0 0 0 0];
dual_F = dual_Fg + dual_Fp;

Jq = [1  0   0  0;
      0  100 0  0;
      0  0  100 0;
      0  0  0  100];
mq = [1  0  0  0;
      0  m  0  0;
      0  0  m  0;
      0  0  0  m];  
  
dual_J = [Jq       zeros(4); 
          zeros(4) mq];

% inv_dual_J = inv(dual_J);

[dual_omega', (dual_J*dual_omega') dual_Fg'];
dq_cross(dual_omega, (dual_J*dual_omega')' )';

r = dq_get_translation_vector(dual_q);
v = dual_omega(6:8);
omega = dual_omega(2:4);
% dual_omega_s = [dual_omega(1:4) 0 v + cross(r,omega)]

% ----------------------------------- %
% Dynamic 9-dof - working!!!
% ----------------------------------- %
% J = Jq(2:4,2:4);
% F = dual_Fg(6:8);
% M = dual_Fg(2:4);
% domega_dt = J\(M') - J\cross(omega', J*omega');
% dv_dt     = [m 0 0; 0 m 0; 0 0 m]\F' - cross(omega', v');
% d_dual_omega_dt = [0; domega_dt; 0; dv_dt];
% [v' omega' cross(omega', m*v')/m J\cross(omega', J*omega')]

% ----------------------------------- %
% Dynamic 10-dof - working!!!
% ----------------------------------- %
% domega_dt = Jq\dual_F(1:4)' - Jq\quatcross(dual_omega(1:4), (Jq*dual_omega(1:4)')')';
% dv_dt     = mq\dual_F(5:8)' - mq\quatcross(dual_omega(1:4), (mq*dual_omega(5:8)')')';
% d_dual_omega_dt = [domega_dt; dv_dt];


% [ (Jq*dual_omega(1:4)') (mq*dual_omega(5:8)')]
% [quatcross(dual_omega(1:4), (Jq*dual_omega(1:4)')')' quatcross(dual_omega(1:4), (mq*dual_omega(5:8)')')']
% dq_cross(dual_omega, (dual_J*dual_omega')')'


% ----------------------------------- %
% Dynamic 12-dof - working!!!
% ----------------------------------- %
d_dual_omega_dt = dual_J\dual_F' - dual_J\dq_cross([dual_omega(1:4) 0 0 0 0], (dual_J*dual_omega')')'; 
d_dual_q_dt     = 0.5*dq_multiply(dual_q, dual_omega);
quatmod(dual_q(1:4));
dx_dt = [d_dual_omega_dt; d_dual_q_dt'];

end

function q_out = quatcross(q, p)
% q = quatmultiply(q, p);
% q_out = [0 q(2) q(3) q(4)];

% q_out = 0.5*(quatmultiply(q, p) - quatmultiply(p, q));

q_out = [0 q(1)*p(2:4) + p(1)*q(2:4) + cross(q(2:4), p(2:4))];
end

function syms_equation()
syms omega_0 omega_x omega_y omega_z;
syms v_0 v_x v_y v_z;
syms r_0 r_x r_y r_z;
syms q_0 q_x q_y q_z;

syms F0 Fx Fy Fz;

syms Jx Jy Jz m;

dual_q     = [q_0 q_x q_y q_z  r_0 r_x r_y r_z];
dual_omega = [omega_0 omega_x omega_y omega_z v_0 v_x v_y v_z];

d_dual_q_dt     = 0.5*dq_multiply(dual_q, dual_omega);

end