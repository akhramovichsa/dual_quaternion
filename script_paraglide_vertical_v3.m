function script_paraglide_vertical_v3()
clc; clear all;

dual_omega = [0 deg2rad(0) deg2rad(0) deg2rad(0)  0 10 0 0];        % Начальная скорость
dual_q     = dq_from_euler_translation(deg2rad([0 0 0]), [0 10 0]); % Начальное положение

options = odeset('RelTol',2e-2);
[t, x] = ode113(@dx_dt, [0:0.5:40], [dual_omega, dual_q], options);

dq = x(:, 9:16);
figure(1)
clf
x_max = 550;
y_max = 130;
% axis([0 x_max -30 y_max])
hold on
grid on
for i = 1:1:length(dq)
    dq_mass = dq(i, :);
    [gamma psi theta] = dq_get_rotation_euler(dq_mass);
    
    % Положение центра масс
    r_mass = dq_get_translation_vector(dq_mass); % Положение центра масс
    % r_mass = dq_transform_vector(r_mass, dq_conj(dq_mass)); % перевести ССК в НСК
    
    % Положение крыла
    dq_wing = dq_multiply(dq_mass, dq_from_euler_translation(deg2rad([0 0 0]), [0 5 0]));
    r_wing  = dq_get_translation_vector(dq_wing); % Положение крыла
    % r_wing  = dq_transform_vector(r_wing, dq_conj(dq_mass)); % перевести ССК в НСК

    dq_ox = dq_multiply(dq_mass, dq_from_euler_translation(deg2rad([0 0 0]), [2 0 0]));
    r_ox  = dq_get_translation_vector(dq_ox); 
    % r_ox  = dq_transform_vector(r_ox, dq_conj(dq_mass)); % перевести ССК в НСК
    
    [rad2deg([gamma psi theta]), r_mass];
    plot(r_mass(1), r_mass(2), 'o');
    plot(r_wing(1), r_wing(2), 's');
    line([r_mass(1) r_wing(1)], [r_mass(2) r_wing(2)])
    
    line([r_mass(1) r_ox(1)], [r_mass(2) r_ox(2)])
    
end
end


function dx_dt = dx_dt(t, x)
dual_omega = x(1:8)';  % Бикватернион угловой и линейной скоростей
dual_q     = x(9:16)'; % Бикватернион положения ЛА вокруг центра масс и центра масс

omega = dual_omega(2:4); % Угловая скорость, [рад/с]
V     = dual_omega(6:8); % Линейная скорость, [м/с]

[gamma psi theta] = dq_get_rotation_euler(dual_q);     % Углы ориентации, [рад]
r                 = dq_get_translation_vector(dual_q); % Радиу-вектор положения ЛА, [м]

alpha = atan(-V(2)/V(1));                 % Угол атаки, [рад]
betta = atan(V(3)/sqrt(V(1)^2 + V(2)^2)); % Угол скольжения, [рад]

g             = 9.807;
Va            = sqrt(V(1)^2 + V(2)^2 + V(3)^2); % Воздушная скорость, [м/с]
rho           = 1.225;                          % Давление атмосферы
q             = 0.5*rho*(Va^2);                 % Скоростной напор
mass_wing     = 0.5;                            % Масса крыла, [кг]
mass_payload  = 3;                              % Масса полезной нагрузки, [кг]
mass = mass_wing + mass_payload;                % Общая масса ЛА, [кг]

S_wing        = 3; % Площадь крыла, [м^2]
ba            = 1; % САХ, [м]

engine_thrust = 0.1*g; % Величина тяги двигателя, [Н]

% -------------------------------------------------------------------------
% Сила и момент силы тяжести крыла
% -------------------------------------------------------------------------
Rg_wing     = [0 3 0];                                  % точка приложения силы в ССК

Fg_wing_nsk = [0 -mass_wing*g 0];                       % сила тяжести в НСК
% Fg_wing     = dq_transform_vector(Fg_wing_nsk, dq_conj(dual_q)); % сила тяжести в ССК
Fg_wing     = Fg_wing_nsk*nsk2ssk(gamma, psi, theta);

Mg_wing     = cross(Rg_wing, Fg_wing);                  % момент в ССК

% -------------------------------------------------------------------------
% Сила и момент силы тяжести полезного груза
% -------------------------------------------------------------------------
Rg_payload     = [0 -0.5 0];                                   % точка приложения силы в ССК

Fg_payload_nsk = [0 -mass_payload*g 0];                       % сила тяжести в НСК
% Fg_payload     = dq_transform_vector(Fg_payload_nsk, dq_conj(dual_q)); % сила тяжести в ССК
Fg_payload     = Fg_payload_nsk*nsk2ssk(gamma, psi, theta);

Mg_payload     = cross(Rg_payload, Fg_payload);               % момент в ССК

% -------------------------------------------------------------------------
% Аэродинамическая сила и момент аэродинамической силы крыла 
% Внимание! Задются в ПССК, затем надо перевести в ССК
% -------------------------------------------------------------------------
% ClarkY
% alpha_interp = deg2rad([-6.0;    -4.5;    -3.0;    -1.5;     0.0;     1.5;     3.0;     6.0;     9.0;     12.0;     15.0;     18.0;     21.0]);
% Cx_interp    =         [ 0.0108;  0.0107;  0.0119;  0.0139;  0.0172;  0.0228;  0.0288;  0.0464;  0.0700;   0.0985;   0.1272;   0.2108;   0.2946];
% Cy_interp    =         [-0.0600;  0.0450;  0.1670;  0.2680;  0.3840;  0.5010;  0.6020;  0.8190;  1.0340;   1.2310;   1.3670;   1.2830;   1.0810];
% Cm_z_interp  =         [ 0.0680;  0.0913;  0.1197;  0.1368;  0.1660;  0.2050;  0.2230;  0.2830;  0.3130;   0.3440;   0.4190;   0.3770;   0.3180];

% ClarkY see http://airfoiltools.com/airfoil/details?airfoil=clarkyh-il#polars
alpha_interp = deg2rad([-60.0;     -17.5;    -15.0;    -10.0;    -5.0;     0.0;      5.0;     10.0;     15.0;     17.5;   60.0]);
Cm_z_0       = 0.0;
Cm_z_alpha   = 0.0;
Cx_interp    =         [  0.035;     0.035;    0.025;    0.02;    0.015;   0.01;     0.015;    0.02;     0.05;     0.1;    0.1];
Cy_interp    =         [ -1.15;     -1.15;    -1.2;     -0.8;    -0.3;     0.25;     0.8;      1.2;      1.2;      1.15;   1.0];
Cm_z_interp  =         [ -0.06;     -0.06;    -0.05;    -0.03;   -0.02;   -0.015;   -0.025;   -0.015;   -0.002;   -0.015; -0.015];
Cd_interp    =         [  0.7;       0.7;      0.7;      0.7;     0.7;     0.432;    0.37;     0.32;     0.3;      0.3;    0.5];

% XFLR5 всего параплана
alpha_interp = deg2rad([-20.0;     -10.0;    -5.0;    0.0;    5.0;     10.0;      20.0]);
Cx_interp    =         [  0.1;       0.04;    0.02;   0.015;  0.03;     0.06;     0.14];
Cy_interp    =         [ -0.9;      -0.4;    -0.15;   0.13;   0.4;      0.7;      1.2];
Cm_z_interp  =         [ -1.1;      -0.3;    -0.05;   0.05;   0.07;    -0.03;    -0.6] - 0.08;
Cd_interp    =         [  0.32;      0.19;     0.1;   0.35;   0.29;     0.24;     0.32];


Cm_z_0=       -0.052;
Cm_z_alpha=     -2.6;
Cx = interp1(alpha_interp, Cx_interp, alpha, 'linear', 'extrap'); % Коэффициент силы сопротивления силы, зависит от угла атаки
Cy = interp1(alpha_interp, Cy_interp, alpha, 'linear', 'extrap'); % Коэффициент подъемной силы, зависит от угла атаки
Cz = 0.0;                                                         % Коэффициент боковой силы, зависит от угла атаки и скольжения
Mz = interp1(alpha_interp, Cm_z_interp, alpha, 'linear', 'extrap')*alpha; % (Cm_z_0 + Cm_z_alpha*alpha);
Cd = interp1(alpha_interp, Cd_interp,   alpha, 'linear', 'extrap'); % Центр давления от передней кромки крыла

Ra_wing     = [-0.35-Cd 3 0]; % точка приложения силы в ССК, зависит от угла атаки

Fa_pssk = [-Cx Cy Cz]*q*S_wing;       % вектор аэродинамической силы в ПССК
Fa = Fa_pssk*pssk2ssk(alpha, betta);  % перевод в ССК

Ma_pssk =  [0 0 Mz]*q*ba*S_wing; % момент в ПССК
% Ma_pssk = cross(Ra_wing, Fa_pssk);    % момент в ПССК
Ma = cross(Ra_wing, Fa);

% Ma = Ma_pssk; %*pssk2ssk(alpha, betta);

% Бикватернион ориентации ПССК относительно ССК
dq_pssk_ssk = dq_from_euler_translation(deg2rad([0 betta alpha]), r);

% Перевод из ПССК в ССК
% Fa          = dq_transform_vector(Fa_pssk, dq_pssk_ssk);
% Ma          = dq_transform_vector(Ma_pssk, dq_pssk_ssk);

% Fa = Fa_pssk*pssk2ssk(alpha, betta);
% Ma = Ma_pssk*pssk2ssk(alpha, betta);
% Ma = cross(Ra_wing, Fa);

% -------------------------------------------------------------------------
%  Сила и момент двигателя тяги
% -------------------------------------------------------------------------
engine_thrust =  0.0; % 1000*(0 - theta) % + 0.5*(100 - r(2));

% engine_thrust = 0.8*(100 - r(2)); % + 10 *(0 - theta);

engine_thrust = 1.5*(18 - V(1)) + 10 *(0 - theta);

k1 = 0.1; % Влияет на колебания
k2 = 0.1;
cross_omega_V = cross(omega, V);
n = 2;

engine_thrust = -mass/mass*( (r(n) - 120)*(1 + k2*k1) + ... 
                        V(n)*(k1 + k2) + ...
                        (1/1)*(Fg_wing(n) + Fg_payload(n) + Fa(n)) + cross_omega_V(n) );


k1 =  [1 0 0; 0 1 0; 0 0 1]*10.2;
k2 =  [1 0 0; 0 1 0; 0 0 1]*30.2;
e = mass*(-(r - [r(1) 20 0])*(eye(3) + k1*k2) ...
          -(V - [0 0 0])*(k1 + k2) ...
          -0*(1/mass)*(Fg_wing + Fg_payload + Fa) + 0*cross(omega, V) );

Fp = e; % [e(1) 0 0];

P_MAX = 40;
if (Fp(1) < 0) Fp(1) = 0; end;
if (Fp(1) > P_MAX) Fp(1) = P_MAX; end;
if (Fp(2) < 0) Fp(2) = 0; end;
if (Fp(2) > P_MAX) Fp(2) = P_MAX; end;

engine_thrust = Fp(1);

Rp = [0.0 -0.2 0.0];       % точка приложения силы в ССК
% Fp = [engine_thrust 0 0]; % сила ССК
Mp = cross(Rp, Fp);       % момент в ССК

% -------------------------------------------------------------------------
% Тензор инерции
% -------------------------------------------------------------------------
Jq = [1  0   0  0;
      0  25  0  0;
      0  0   1  0;
      0  0   0  25];
mq = [1   0    0    0;
      0  mass  0    0;
      0   0   mass  0;
      0   0    0   mass]; 
  
dual_J = [  Jq       zeros(4); 
          zeros(4)     mq];

% J = Jq(2:4,2:4);
% f1 = inv(J)*(cross(omega', J*omega') + Fg_wing' + Fg_payload');
% 
% k1 = [1 1 1 1]*0.1;
% k2 = [1 1 1 1]*1.8;
% 
% Mp = - Jq*(0.5*quatmultiply(dual_q(1:4), dual_q(1:4)) + [0 f1'] + quatmultiply(k2, [0 omega]) + 2*quatmultiply(k2, k1))';
% 
% Mp = - Jq*(0.5*quatmultiply(dual_q(1:4), dual_q(1:4)) + [0 f1'] + k2(1)*[0 omega] + [2*k2(1)*k1(1) 2*k2(1)*k1(1) 2*k2(1)*k1(1) 2*k2(1)*k1(1)])';
% 
% Mp = [ 0 0 Mp(4)']
% 
% if (Mp(3) < 0) Mp(3) = 0; end;
% -------------------------------------------------------------------------
% Сумма моментов и сил в бикватернионной форме
% -------------------------------------------------------------------------
F = [0 0 0];
F = F + Fg_wing;
F = F + Fg_payload;
F = F + Fa;
F = F + Fp;

M = [0 0 0];
% M = M + Mg_wing;
% M = M + Mg_payload;
% M = M + Ma;
% M = M + Mp;

% Неуправляемые моменты и силы в бикватернионах
dual_F = [0 M 0 F];

% Управляемые моменты и силы в бикватернионах
% dual_F_control = [0 Mp 0 Fp];

% k = [0 0 0 deg2rad(2)  0 0 0 0];
% omega(3);
% dual_F_control = -dual_F + dq_cross([dual_omega(1:4) 0 0 0 0], (dual_J*dual_omega')') - (dual_J*dq_multiply(k, dual_omega)')'

% Общие моменты и силы в бикватернионах
% dual_F = dual_F + dual_F_control;

disp([num2str(t, '%10.2f:'), char(9), ...
      'alpha:', char(9), num2str(rad2deg(alpha), '%10.2f'), char(9), ...
      'theta:', char(9), num2str(rad2deg(theta), '%10.2f'), char(9), ...
      'F:',     char(9), num2str(F(1:2), '%10.2f'), char(9), ...
      'Ma_z:',  char(9), num2str(M(3), '%10.2f'), char(9), char(9), ...
      'V:',     char(9), num2str(V(1:2), '%10.2f'), char(9), ...
      'H:',     char(9), num2str(r(2), '%10.2f'), char(9), char(9), ...
      'engine:',char(9), num2str(Fp(1:2), '%10.2f'), char(9), char(9)] ...
  );

% -------------------------------------------------------------------------
% Динамические уравнения движения в бикватернионах
% -------------------------------------------------------------------------
d_dual_omega_dt = dual_J\dual_F' - dual_J\dq_cross([dual_omega(1:4) 0 0 0 0], (dual_J*dual_omega')')'; 
d_dual_q_dt     = 0.5*dq_multiply(dual_q, dual_omega);

% if (abs(rad2deg(alpha)) > 15) return; end;
% if (abs(rad2deg(theta)) > 40) return; end;

dx_dt = [d_dual_omega_dt; d_dual_q_dt'];


% -------------------------------------------------------------------------
% Динамические уравнения движения обычные
% -------------------------------------------------------------------------
% J = Jq(2:4,2:4);
% J_inv = inv(J);
% domega_dt = J_inv*((M') - cross(omega', J*omega'));
% dv_dt     = [mass 0 0; 0 mass 0; 0 0 mass]\F' - cross(omega', V');
% 
% d_dual_omega_dt = [0; domega_dt; 0; dv_dt];
% d_dual_q_dt     = 0.5*dq_multiply(dual_q, dual_omega);
% 
% dx_dt = [d_dual_omega_dt; d_dual_q_dt'];


end


function M = pssk2ssk(alpha, betta)
% http://lektsii.org/3-22923.html
sa = sin(alpha);
ca = cos(alpha);
sb = sin(betta);
cb = cos(betta);

M = [ ca*cb sa -ca*sb;
     -sa*cb ca  sa*sb;
      sb     0  cb]';

end

function M = nsk2ssk(gamma, psi, theta)
sin_gamma = sin(gamma);
cos_gamma = cos(gamma);

sin_psi = sin(psi);
cos_psi = cos(psi);

sin_theta = sin(theta);
cos_theta = cos(theta);

M = [cos_psi*cos_theta                                 sin_theta            -sin_psi*cos_theta
    -cos_psi*sin_theta*cos_gamma + sin_psi*sin_gamma   cos_theta*cos_gamma   cos_psi*sin_gamma + sin_psi*sin_theta*cos_gamma
     cos_psi*sin_theta*sin_gamma + sin_psi*cos_gamma  -cos_theta*sin_gamma   cos_psi*cos_gamma - sin_psi*sin_theta*sin_gamma]';
end