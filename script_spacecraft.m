function script_spacecraft()
clc; clear all;
addpath('orb','-end');

global c;
c.EARTH_MU = 398600.4415; % Геоцентрическая гравитационная постоянная, [км^3/c^2]
c.EARTH_R0 = 6378;

% Начальные условия полжения и ориентации КА
rv = orb_KeplerToXYZ(c.EARTH_R0 + 622, 0.0, deg2rad(30.0), deg2rad(0.0), deg2rad(0.0), c.EARTH_MU);

psi     = deg2rad(0);
theta   = deg2rad(0);
gamma   = deg2rad(0);
omega_x = deg2rad(0);
omega_y = deg2rad(0);
omega_z = deg2rad(0);

% Перевод в бикватернионную форму
dual_omega = [0 omega_x omega_y omega_z  0 rv(4:6)'];                  % Начальная угловая и линейная скорости
dual_q     = dq_from_euler_translation([psi theta gamma], rv(1:3)'); % Начальная ориентация и положение ЛА

% Интегрирование системы
options = odeset('RelTol', 2e-6);
[t, x] = ode45(@dx_dt, [0:10.0:2*3600], [dual_omega, dual_q], options);

d_omega = x(:, 1:8);
d_q     = x(:, 9:16);


figure(1)
% clf
x_max = 550;
y_max = 130;
% axis([0 x_max -30 y_max])
hold on
grid on
% figure('Color', [1 1 1]);
xlabel('\it r_x ,  м');
ylabel('\it r_z ,  м');
zlabel('\it r_y ,  м');
for i = 1:1:length(d_q)
    r                 = dq_get_translation_vector(d_q(i,:)); % Положение центра масс
    rr(i, :) = r;
    
    plot3(r(1), r(2), r(3), 'k.'); % , LineStyle, '--', LineWidth, 1.0);
end

[x, y, z] = sphere;

surf(c.EARTH_R0.*x, c.EARTH_R0.*y, c.EARTH_R0.*z);

line_length = c.EARTH_R0 + 100;
line([-line_length, line_length], [0, 0],                      [0, 0],                      'Color', [1 0 0]);
line([0 0 ],                      [-line_length, line_length], [0, 0],                      'Color', [0 1 0]);
line([0, 0],                      [0, 0],                      [-line_length, line_length], 'Color', [0 0 1]);


end

function dx_dt = dx_dt(t, x)
global c;

dual_omega = x(1:8)';  % Бикватернион угловой и линейной скоростей
dual_q     = x(9:16)'; % Бикватернион положения ЛА вокруг центра масс и центра масс

omega = dual_omega(2:4); % Угловая скорость, [рад/с]
V     = dual_omega(6:8); % Линейная скорость, [м/с]

[psi theta gamma] = dq_get_rotation_euler(dual_q);     % Углы ориентации, [рад]
r                 = dq_get_translation_vector(dual_q); % Радиус-вектор положения ЛА, [м]

% -------------------------------------------------------------------------
% Константы и характеристики КА
% -------------------------------------------------------------------------
mass     = 1;         % Масса КА, [кг]

% -------------------------------------------------------------------------
% Тензор инерции
% -------------------------------------------------------------------------
Jq = [1  0       0        0;
      0  3.670   0        0;
      0  0       3.710    0;
      0  0       0        0.104];
mq = [1   0    0    0;
      0  mass  0    0;
      0   0   mass  0;
      0   0    0   mass]; 
  
dual_J = [  Jq       zeros(4); 
          zeros(4)     mq];

% -------------------------------------------------------------------------
% Гравитационная сила
% -------------------------------------------------------------------------
Fg_nsk = -c.EARTH_MU*r/(norm(r)^3);
% Fg     = Fg_nsk*nsk2ssk(gamma, psi, theta);

% -------------------------------------------------------------------------
% Сумма сил
% -------------------------------------------------------------------------
dual_F = [0 0 0 0 0 Fg_nsk];

% -------------------------------------------------------------------------
% Уравнения движения
% -------------------------------------------------------------------------
d_dual_omega_dt = dual_J\dual_F' - dual_J\dq_cross([dual_omega(1:4) 0 0 0 0], (dual_J*dual_omega')')'; 
d_dual_q_dt     = 0.5*dq_multiply(dual_q, dual_omega);

dx_dt = [d_dual_omega_dt; d_dual_q_dt'];


end