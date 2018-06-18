function script_for_article_3_v2()
clc; clear all;

global p; % Глобальные параметры
global gt gu gi;

% temp_cx();
% return;

% Турбулентность, модель Драйдена
% syms Va delta_x delta_y delta_z Lx Ly Lz;
% Hx = ilaplace(delta_x*sqrt(2*Va/Lx) / (s+Va/Lx))
% Hy = ilaplace(delta_y*sqrt(3*Va/Ly) * (s + Va/(sqrt(3)*Ly)) / (s+Va/Ly)^2 )

% t = 0:0.1:10
% plot(t, randn(1, length(t)));
% return;

% -------------------------------------------------------------------------
% Константы и характеристики ЛА
% -------------------------------------------------------------------------
p.g_0    = 9.807; % Ускорение свободного падения, [м*с^2]
p.rho_0  = 1.225; % Давление атмосферы на уровне моря при t=+15, [кг/м^3]

p.mass   = 1;     % Масса ЛА, [кг]
p.l_wing = 2;     % Размах (длина) крыла, [м]
p.S      = 2;     % Площадь крыла, [м^2]
p.ba     = 1;     % САХ
% p.Jzz    = 0.37;  % При массе ПН 1 кг, 2 метра под крылом    % Момент инерции вокруг оси OZ
p.ly     = -0.1;  % Точка положения двигателя тяги относительно центра масс ЛА по оси OY, [м]

% Управление
p.u_delta_left  = deg2rad(0); % Левая клеванта
p.u_delta_right = deg2rad(0); % Правая клеванта
p.u_thrust      = 0;          % Тяга двигателя
p.u             = false;      % Включить управление

p.u_delta_x     = deg2rad(0); % Управление через моменты
p.u_delta_y     = deg2rad(0); % Управление через моменты
p.u_delta_z     = deg2rad(0); % Управление через моменты

% Ветер
p.Wind     = [0.0; 0.0; 0.0];       % Постоянный ветер, [м/с]. Направления: [юг->север; снизу->вверх; запад->восток]
p.Wind_tur = [false; false; false]; % Учет турбулентности в ССК

% Тензор инерции
Jq = [1  0       0        0;
      0  3.670   0        0;
      0  0       3.710    0;
      0  0       0        0.104];

mq = [1    0      0      0;
      0  p.mass   0      0;
      0    0    p.mass   0;
      0    0      0    p.mass]; 
  
p.dual_J = [  Jq       zeros(4); 
            zeros(4)     mq];

% -------------------------------------------------------------------------
% Начальные условия полжения и ориентации ЛА
% -------------------------------------------------------------------------
rx      = 0;
ry      = 0;
rz      = 0;
Vx      = 10;
Vy      = 0;
Vz      = 0;

psi     = deg2rad(0);
theta   = deg2rad(10);
gamma   = deg2rad(0);
omega_x = deg2rad(0);
omega_y = deg2rad(0);
omega_z = deg2rad(0);
% u       = 0;

% Перевод в бикватернионную форму
dual_omega = [0; omega_x; omega_y; omega_z;  0; Vx; Vy; Vz];               % Начальная угловая и линейная скорости
dual_q     = dq_from_euler_translation([psi; theta; gamma], [rx; ry; rz]); % Начальная ориентация и положение ЛА

% -------------------------------------------------------------------------
% Интегрирование системы
% -------------------------------------------------------------------------
tspan = 0:0.05:20;
x = ode4(@dx_dt, tspan, [dual_omega; dual_q]);

% options = odeset('RelTol', 1e-2);
% [t, x] = ode45(@dx_dt, tspan, [dual_omega; dual_q], options);

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
    r                 = dq_get_translation_vector(d_q(i,:)'); % Положение центра масс
    rr(i, :) = r;
    [psi, theta, gamma] = dq_get_rotation_euler(d_q(i,:)');     % Ориентация ЛА
    phi(i,:) = [gamma psi theta];
    
    % plot(r(1), r(2), 'o');
    plot3(r(1), r(3), r(2), 'k.'); % , LineStyle, '--', LineWidth, 1.0);
    line([r(1), r(1)], [r(3), r(3)], [r(2), 0]);
end

line([-50, 50], [0, 0],    [0, 0],    'Color', [1 0 0]);
line([0 0 ],    [-50, 50], [0, 0],    'Color', [0 1 0]);
line([0, 0],    [0, 0],    [-50, 50], 'Color', [0 0 1]);

% plot_gost(t,  rad2deg(x(:,2:4)), '\it t,  с', '\rm \omega ,\it  град/с',         'fig_omega_t');
% legend('\omega_{\it x}', '\omega_{\it y}', '\omega_{\it z}', 'Location', 'northeast', 'Orientation', 'horizontal');

% plot_gost(t,  x(:,6:8), '\it t,  с', '\bf V ,\rm \it  м/с',         'fig_v_t');
% legend('\it V_x', '\it V_y', '\it V_z', 'Location', 'northeast', 'Orientation', 'horizontal');

% plot_gost(t,  x(:,9:12), '\it t,  с', '\bf q_1',         'fig_q1_t');
% legend('\it q_0', '\it q_1', '\it q_2', '\it q_3', 'Location', 'northeast', 'Orientation', 'horizontal');

% plot_gost(t,  x(:,13:16), '\it t,  с', '\bf q_2',         'fig_q2_t');
% legend('\it q_0', '\it q_1', '\it q_2', '\it q_3', 'Location', 'northeast', 'Orientation', 'horizontal');

% plot_gost(t,  rr(:,1:3), '\it t,  с', '\bf r ,\rm \it  м/с ',         'fig_r_t');
% legend('\it r_x', '\it r_y', '\it r_z', 'Location', 'northeast', 'Orientation', 'horizontal');

% plot_gost(t,  rad2deg(phi(:,1:3)), '\it t,  с', '\gamma, \psi, \vartheta,  \it  град/с ',         'fig_phi_t');
% legend('\gamma', '\psi', '\vartheta', 'Location', 'northeast', 'Orientation', 'horizontal');

end

function dx_dt = dx_dt(t, x)
global p;

dual_omega = x(1:8);  % Бикватернион угловой и линейной скоростей
dual_q     = x(9:16); % Бикватернион положения ЛА вокруг центра масс и центра масс

omega = dual_omega(2:4); % Угловая скорость, [рад/с]
V     = dual_omega(6:8); % Линейная скорость, [м/с]

% ppsi = rad2deg(atan(2*(dual_q(1)*dual_q(3) - dual_q(2)*dual_q(4)/quatnorm(dual_q(1:4)))))
[psi theta gamma] = dq_get_rotation_euler(dual_q);     % Углы ориентации, [рад]
r                 = dq_get_translation_vector(dual_q); % Радиус-вектор положения ЛА, [м]

% -------------------------------------------------------------------------
% Ветер и турбулентность
% -------------------------------------------------------------------------
W_0_nsk = p.Wind; % Направления: [юг->север; снизу->вверх; запад->восток]
W_0     = nsk2ssk(gamma, psi, theta)*W_0_nsk;
s = rng;
W_tur   = [0.0; 0.0; 0*3*sin(t)];

W = W_0 + W_tur;

% Дополнительные параметры
Va      = V - W;
Va_norm = sqrt(Va(1)^2 + Va(2)^2 + Va(3)^2); % Воздушная скорость, [м/с]
q       = (p.rho_0*Va_norm^2)/2;                 % Скоростной напор, [кг/(м*с^2)]
alpha   = atan(-Va(2)/Va(1));                % Угол атаки, [рад]
betta   = asin(Va(3)/Va_norm);               % Угол скольжения, [рад]

% -------------------------------------------------------------------------
% Управляющее воздействие клевантами
% -------------------------------------------------------------------------
% Управление стропами, рули отклоняются только вниз в полжительном направлении от 0 до 20 градусов.
% delta_left  = deg2rad(0.0); % Угол отклонения левого руля, [рад]
% delta_right = deg2rad(0.0); % Угол отклонения правого руля, [рад]

% Управляющие воздействия вокруг по каждой из осей, [рад]
delta_x = 0.5*(p.u_delta_left  - p.u_delta_right);
delta_y = 0.5*(p.u_delta_right - p.u_delta_left);
delta_z = 0.5*(p.u_delta_left  + p.u_delta_right);

% -------------------------------------------------------------------------
% Аэродинамическая сила
% -------------------------------------------------------------------------
% Аэродинамические коэфициенты
cx_alpha_0 = 0.008;
cx_alpha   = 0.05;
cx_alpha_2 = 1.1;
cx_delta   = 0*0.01;

cy_alpha_0 = 0.1 + 0.02;
cy_alpha   = 2.3;
cy_delta   = 0*0.2;

cz_betta_0 =  0.0;
cz_betta   = -0.11;
cz_delta   =  0*0.01;

mx_betta_0 = 0.0;
mx_betta   = 0.0;
mx_delta   = 0.01;

my_betta_0 =  0.0;
my_betta   = -0.001;
my_delta   =  0.003;

mz_alpha_0 = -0.035;
mz_alpha   = -1.0;
mz_alpha_2 = -10.0;
mz_delta   =  0.01;

cx = cx_alpha_0 + cx_alpha*alpha + cx_alpha_2*alpha^2 + cx_delta*p.u_delta_x;
cy = cy_alpha_0 + cy_alpha*alpha + cy_delta*p.u_delta_z;
cz = cz_betta_0 + cz_betta*betta + cz_delta*p.u_delta_y;

mx = mx_betta_0 + mx_betta*betta + mx_delta*p.u_delta_x;
my = my_betta_0 + my_betta*betta + my_delta*p.u_delta_y;
mz = mz_alpha_0 + mz_alpha*alpha + mz_delta*p.u_delta_z; % + mz_alpha_2*alpha^2 ;

Fa_pssk = [-cx; cy; cz]*q*p.S;          % вектор аэродинамической силы в ПССК

Fa = pssk2ssk(alpha, betta)*Fa_pssk;  % перевод в ССК
Ma =  [mx; my; mz]*q*p.ba*p.S;              % момент в ПССК

[t Fa' rad2deg(alpha) rad2deg(betta) mz];

% -------------------------------------------------------------------------
% Сила тяжести
% -------------------------------------------------------------------------
Fg_nsk = [0; -p.mass*p.g_0; 0];
Fg     = nsk2ssk(gamma, psi, theta)*Fg_nsk;

% -------------------------------------------------------------------------
% Сила и момент тяги двигателя
% -------------------------------------------------------------------------
Rp      = [-0.1; -0.2; 0.0];  % точка приложения силы в ССК

% engine_thrust =  0.0; % 1000*(0 - theta) % + 0.5*(100 - r(2));

Fp = [p.u_thrust; 0; 0]; % сила ССК
Mp = cross(Rp, Fp);         % момент в ССК

Mp = Mp + [0; -0*p.u_thrust*0.01; 0]; % Момент от винта

% -------------------------------------------------------------------------
% Сумма сил и моментов в бикватернионной форме
% -------------------------------------------------------------------------
F = [0; 0; 0];
F = F + Fg;
F = F + Fa;
F = F + Fp;

M = [0; 0; 0];
M = M + Ma;
M = M + Mp;

% Моменты и силы в бикватернионной форме
dual_F = [0; M; 0; F];

% -------------------------------------------------------------------------
% Регулятор
% -------------------------------------------------------------------------
r_tr      = [r(1)+10; 20; r(3)];
dual_q_tr = dq_from_euler_translation([psi theta gamma], r_tr);
dual_qe = dq_multiply(dq_inv(dual_q_tr), dual_q); % - dual_q_one;

dual_right_part = p.dual_J \ ( dual_F - dq_cross([dual_omega(1:4); 0; 0; 0; 0], (p.dual_J*dual_omega)) );
cK = 0.01;
cN = 2.5;

% Работает отлично, результат из диссертации k = 0.01 n = 2.5
dual_Fu = p.dual_J*(- dq_multiply(dual_qe, (dual_qe + cK*cN*dq_inv(dual_qe)) ) ...
                    - cN*dual_omega  ...
                    - dual_right_part );


if (p.u == true)
    % Тяга двигателя
    p.u_thrust = dual_Fu(4)/Rp(2);
%     p.u_thrust = -dual_Fu(6);
p.u_thrust
    if (p.u_thrust < 0) p.u_thrust = 0; end;
    if (p.u_thrust > 5) p.u_thrust = 5; end;
    
    % Управление моментами
    delta_max = deg2rad(10);
    p.u_delta_x = dual_Fu(2)/(mx_delta*q*p.S*p.ba); % Момент
    if (p.u_delta_x < -delta_max) p.u_delta_x = -delta_max; end;
    if (p.u_delta_x >  delta_max) p.u_delta_x =  delta_max; end;
    
    p.u_delta_y = dual_Fu(3)/(my_delta*q*p.S*p.ba); % Момент
    if (p.u_delta_y < -delta_max) p.u_delta_y = -delta_max; end;
    if (p.u_delta_y >  delta_max) p.u_delta_y =  delta_max; end;

    p.u_delta_z = dual_Fu(4)/(mz_delta*q*p.S*p.ba); % Момент
    if (p.u_delta_z < -delta_max) p.u_delta_z = -delta_max; end;
    if (p.u_delta_z >  delta_max) p.u_delta_z =  delta_max; end;
    
    u_delta_x_f = dual_Fu(6)/(cx_delta*q*p.S);
    u_delta_y_f = dual_Fu(7)/(cz_delta*q*p.S);
    u_delta_z_f = dual_Fu(8)/(cy_delta*q*p.S);
    % [p.u_delta_x p.u_delta_y p.u_delta_z u_delta_x_f u_delta_y_f u_delta_z_f]
    p.u_delta_x = deg2rad(0);
    p.u_delta_y = deg2rad(0);
    % p.u_delta_z = deg2rad(0);
%     if (delta_y > 0)
%         p.u_delta_left  = 0;
%         p.u_delta_right = delta_y;
%         if (p.u_delta_right < 0)           p.u_delta_right = 0; end;
%         if (p.u_delta_right > deg2rad(10)) p.u_delta_right = deg2rad(10); end;
%     end
    % delta_y = dual_Fu(8)/(cy_delta*q*p.S);      % Сила
end

if ( ~isnan(dual_Fu(1)) && p.u ~= true)
%     dual_Fu(4) = min(10, max(0, dual_Fu(4)));
%     dual_Fu(6) = min(10, max(-10, dual_Fu(4)));
%     dual_Fu(7) = min(10, max(-10, dual_Fu(4)));
    
    dual_Fu = min(10, max(-10, dual_Fu));
%     [dual_Fu(2) dual_Fu(3) dual_Fu(4); dual_Fu(6) dual_Fu(7) dual_Fu(8)];
    dual_F = dual_F + [0; dual_Fu(2); dual_Fu(3); dual_Fu(4);  0; dual_Fu(6); dual_Fu(7); dual_Fu(8)];
end

%       'Fu:',    char(9), num2str(dual_Fu', '%10.2f'), char(9), ...
%       'alpha:', char(9), num2str(rad2deg(alpha), '%10.2f'), char(9), ...
%       'betta:', char(9), num2str(rad2deg(betta), '%10.2f'), char(9), ...
%       'F:',     char(9), num2str(F, '%10.2f'), char(9), ...
%       'M:',     char(9), num2str(M, '%10.2f'), char(9), char(9), ...

disp([num2str(t, '%10.2f:'), char(9), ...
      'V:',     char(9), num2str(V', '%10.2f'), char(9), ...
      'r:',     char(9), num2str(r', '%10.2f'), char(9), char(9), ...
      'omega:',  char(9), num2str(rad2deg(omega'), '%10.2f'), char(9), ...
      'psi, theta, gamma:', char(9), num2str(rad2deg(psi),   '%10.2f'), ' ', ... 
                                     num2str(rad2deg(theta), '%10.2f'), ' ', ...
                                     num2str(rad2deg(gamma), '%10.2f'), char(9), ...
      'control:',char(9), num2str(p.u_thrust,               '%10.2f'), ' ', ...
                          num2str(rad2deg(p.u_delta_left),  '%10.2f'), ' ', ...
                          num2str(rad2deg(p.u_delta_right), '%10.2f'), ' ', ...
                 char(9), char(9)] ...
  );

% -------------------------------------------------------------------------
% Уравнения движения
% -------------------------------------------------------------------------
d_dual_omega_dt = p.dual_J \ (dual_F - dq_cross([dual_omega(1:4); 0; 0; 0; 0], (p.dual_J*dual_omega))); 
d_dual_q_dt     = 0.5*dq_multiply(dual_q, dual_omega);

dx_dt = [d_dual_omega_dt; d_dual_q_dt];

end





% dual_q_epsilon = dq_multiply(dual_q, dq_inv(dual_q_tr));
% dual_q_epsilon =  dual_q - dual_q_tr;


% % dual_omega_tr      = dq_from_euler_translation(deg2rad([0 0 0]), [V(1) 0 0]);
% dual_omega_tr      =  [0; omega;  0; V(1); V(2); V(3)];
% dual_omega_epsilon = dual_omega - dual_omega_tr;
% % dual_omega_epsilon = dq_multiply(dual_omega_tr, dq_inv(dual_omega));

% dual_omega_K = dual_omega_epsilon + dq_multiply(dq_inv(dual_q_epsilon), dual_K, dual_q_epsilon); % dual_K;
% dual_omega_Kq = dual_omega_epsilon + dq_multiply(dual_K, dual_q_epsilon);

% dq_multiply(dq_inv(dual_q), dual_K, dual_q);
% dual_K;
% dual_q;
% dq_get_translation_vector(dual_omega_epsilon);

% dual_Fu = dual_J*(- dq_multiply(dq_inv(dual_omega_K), dual_q_epsilon, dual_q_epsilon, dual_omega_K) ... 
%                   - dq_multiply(dual_N, dual_omega_K)... 
%                   - dual_right_part)';

% dual_Fu =  dual_J*(- dq_multiply(dual_q_epsilon, dual_q_epsilon) - dual_N - dual_right_part)';

% dual_Fu = dual_J*(- dq_multiply(dual_q_epsilon, ([1 ones(1, 7)] + dq_multiply(dual_K, dual_N))) ...
%                   - dq_multiply(dual_omega, (dual_K + dual_N)) ...
%                   - dual_right_part )';

% Классический рабочий регулятор Fu = J*(-(1 + N*K)*q - (k*0.5*q + N)*omega - f)
% dual_Fu = dual_J*(- dq_multiply(([1 zeros(1, 7)] + dq_multiply(dual_N, dual_K)), dual_q_epsilon) ...
%                   - dq_multiply((dq_multiply(dual_K, 0.5*dual_q_epsilon) + dual_N), dual_omega_epsilon) ...
%                   - dual_right_part )';

% Классический рабочий регулятор с константами K и N: Fu = J*(-(1 + N*K)*q - (k*0.5*q + N)*omega - f)
% dual_Fu = dual_J*(- (1 + cN*cK)*dual_q_epsilon ...
%                   - (cK + cN)*dual_omega_epsilon  ...
%                   - dual_right_part )';
%                   - dq_multiply((0.5*cK*dual_omega_epsilon + cN), dual_omega_epsilon) ...
       
% dual_Fu = dual_J*(- (dq_multiply(dual_q_epsilon, dual_omega_epsilon) + dq_multiply(dual_q_epsilon, cK*dual_I)) ...
%                   - cN*dual_omega_epsilon - cN*cK*dual_I ...
%                   - dual_right_part)';

% dual_Fu = dual_J*(- (1 + cN*cK)*dual_q_epsilon...
%                   - cN*dual_omega_epsilon - cK ...
%                   - dual_right_part )';

% dual_Fu = dual_J*(- dq_multiply(dual_omega_Kq, dual_q_epsilon, dual_omega_Kq) ...
%                   - dq_multiply(dual_N, dual_K, dual_q_epsilon) ...
%                   - dq_multiply((dq_multiply(dual_K, 0.5*dual_q) + dual_N), dual_omega_epsilon) ...
%                   - dual_right_part )';

% -------------------------------------------------------------------------
% Регулятор в бикватернионной форме -  НЕ РАБОТАЕТ!!!
% -------------------------------------------------------------------------
% dual_K = 1.1*[1; 1; 1; 1;  1; 1; 1; 1]; % Коэффициент кватернионного углового положения и вектора положения в пространстве
% dual_K = [0.0 0.1 0.1 0.1  0.0 0.1 0.1 0.1];
% dual_K = dq_from_euler_translation(deg2rad([0 0 0]), [0 0 0]);

% dual_N = 0.01*[1; 1; 1; 1;  1; 1; 1; 1]; % Коэффициент угловой скорости и линейной скорости
% dual_N = [0 0.001 0.001 0.001  0 0.001 0.001 0.001];
% dual_N = dq_from_euler_translation(deg2rad([0 0 0]), [0 0 0]);

% r_tr = [r(1)+10; 40; r(3)];
% if (t >= 0 && t < 40)
%     r_tr = [r(1) 40 r(3)];
% end
% 
% if (t >= 40 && t < 80)
%     r_tr = [r(1) 40 100];
% end
% 
% if (t >= 80)
%     r_tr = [r(1) 40 100];
% end

% if (t >= 90)
%     r_tr = [0 40 0];
% end




% Не работает
% dual_Fu = dual_J*(- (1 + cN*cK)*dq_multiply(dual_qe, dual_q) ...
%                   - cN*dual_omega  ...
%                   - dual_right_part );

% Работает отлично              
% dual_Fu = dual_J*(- (1 + cN*cK)*dual_qe ...
%                   - dq_multiply((cN*dual_q_one + dq_inv(dual_omega)*cK), dual_omega)  ...
%                   ... % - dq_multiply(dual_qe*cK, dual_omega) - cN*dual_omega ...
%                   - dual_right_part );

% Не работает
% dual_Fu = dual_J*(- cK*dq_multiply(dq_inv(dual_qe), (dq_inv(dual_qe) + dual_qe - dual_q_one) ) ...
%                   - cN*dual_omega  ...
%                   - dual_right_part );

% Работает низкая точность
% dual_Fu = dual_J*(- dq_multiply(dual_qe, (dual_qe + cN*cK*dual_q_one))...
%                     ... % - cN*cK*dual_q_one - dq_multiply(dual_qe, dual_qe)...
%                   - cN*dual_omega  ...
%                   - dual_right_part );

% Работает отлично 
% dual_Fu = dual_J*(- dq_multiply((dual_qe - dual_q_one), (dual_qe  + cN*cK*dq_inv(dual_qe)))...
%                   ... %+ dq_multiply(dq_inv(dual_qe),dq_inv(dual_qe)) ...
%                   - cN*dual_omega  ...
%                   - dual_right_part );

% PD-control - работает, недолет по высоте
% dual_Fu = - 10.5*dual_qe - 10.1*dual_omega;

% Не работает
% dual_Fu = dual_J*(- dq_multiply(dq_conj(dual_q_epsilon), dual_q) ...
%                   - cN*dual_omega_epsilon - 0*cN*cK*dual_I ...
%                   - dual_right_part );

% dual_Fu = - [0.0 0.1 0.1 0.1  0.0 0.1 0.1 0.1]'; % dq_multiply([0.3 0.3 0.3 0.3  0.1 0.1 0.1 0.1], dual_q_epsilon)'

% -------------------------------------------------------------------------
% Регулятор по угловой и линейной скоростям в бикватернионной форме
% -------------------------------------------------------------------------
% dual_K = 0.1*[1 1 1 1  1 1 1 1];
% 
% dual_omega_tr      = [0 deg2rad([0 0 0])  0 V(1) V(2) V(3)];
% % dual_omega_epsilon = dq_multiply(dual_omega_tr, dq_inv(dual_omega));
% % dual_omega_epsilon = dq_multiply(dual_omega, dq_inv(dual_omega_tr));
% dual_omega_epsilon =  dual_omega - dual_omega_tr;
% 
% dual_right_part = dual_J\dual_F' - dual_J\dq_cross([dual_omega_epsilon(1:4) 0 0 0 0], (dual_J*dual_omega_epsilon')')';
% dual_right_part = dual_right_part';
% 
% dual_Fu = dual_J*(- dq_multiply(dual_K, dual_omega_epsilon) - dual_right_part)';