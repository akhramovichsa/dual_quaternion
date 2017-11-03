function  script_for_article_1()
clc; clear all;
global gt gu gi;
gi = 1;
 
% plotCxCyCd();
% return;

% Начальные условия полжения и ориентации ЛА
rx      = 0;
ry      = 100;
Vx      = 10;
Vy      = 0;
theta   = deg2rad(0);
omega_z = deg2rad(0);
u       = 0;

options = odeset('RelTol',2e-2);
[t, x] = ode113(@dx_dt, [0:0.1:30], [rx; ry; Vx; Vy; theta; omega_z; u], options);

x;

plot_gost(x(:,1),          x(:,2), '\it r_x ,  м', '\it r_y ,  м',         'fig_h_x');

% plot_gost(t,          x(:,2), '\it t,  с', '\it r_y ,  м',         'fig_h_t');
% plot_gost(t,          x(:,7), '\it t,  с', '\it u ,  Н',           'fig_u_t');
% plot_gost(gt,             gu, '\it t,  с', '\it F_p_x ,  Н',       'fig_Fpx_t');
% plot_gost(t, rad2deg(x(:,5)), '\it t,  с', '\vartheta, \it град',  'fig_theta_t');
% plot_gost(t, rad2deg(x(:,6)), '\it t,  с', '\omega_z, \it град/с', 'fig_omega_t');


end

function dx_dt = dx_dt(t, x)
global gt gu gi;

rx      = x(1);
ry      = x(2);
Vx      = x(3);
Vy      = x(4);
theta   = x(5);
omega_z = x(6);
u       = x(7);

% Константы и характеристики ЛА
g    = 9.807; % Ускорение свободного падения, [м*с^2]
rho  = 1.225; % Давление атмосферы на уровне моря при t=+15, [кг/м^3]
mass = 1;     % Масса ЛА, [кг]
l_wing = 2;   % Размах (длина) крыла, [м]
S    = 2;     % Площадь крыла, [м^2]
ba   = 1;     % САХ
Jzz  = 0.37;  % При массе ПН 1 кг, 2 метра под крылом    % Момент инерции вокруг оси OZ
ly   = -0.1;  % Точка положения двигателя тяги относительно центра масс ЛА по оси OY, [м]

% Дополнительные параметры
alpha = atan(-Vy/Vx);      % Угол атаки, [рад]
V     = sqrt(Vx^2 + Vy^2); % Воздушная скорость, [м/с]
q     = (rho*V^2)/2;       % Скоростной напор, [кг/(м*с^2)]

if (abs(alpha) > deg2rad(20))
    disp('ERROR: alpha > +-20')
    return
end
% -------------------------------------------------------------------------
% Аэродинамическая сила
% -------------------------------------------------------------------------
% alpha_interp = deg2rad([-20.0;     -10.0;    -5.0;    0.0;    5.0;     10.0;      20.0]);
% Cx_interp    =         [  0.1;       0.04;    0.02;   0.015;  0.03;     0.06;     0.14];
% Cy_interp    =         [ -0.9;      -0.4;    -0.15;   0.13;   0.4;      0.7;      1.2];
% Cm_z_interp  =         [ -1.1;      -0.3;    -0.05;   0.05;   0.07;    -0.03;    -0.6]; % - 0.08;
% Cd_interp    =         [  0.32;      0.19;     0.1;   0.35;   0.29;     0.24;     0.32];

% CLARK-YH Справочник авиапрофилей стр. 289
% alpha_interp = deg2rad([-20;    -16;    -12;    -8;     -4;     -2;      0;      2;      4;      8;      10;     12;     16;     18;     20;    22;     24]);
% Cx_interp    =         [ 0.257;  0.2028; 0.0948; 0.0254; 0.0126; 0.0116; 0.0126; 0.0162; 0.0226; 0.0428; 0.0592; 0.0768; 0.1176; 0.1462; 0.18;  0.2386; 0.2892];
% Cy_interp    =         [-0.576; -0.596; -0.562; -0.388; -0.13;   0;      0.13;   0.266;  0.4;    0.656;  0.792;  0.924;  1.166;  1.258;  1.28;  1.24;   1.148];
% Cm_z_interp  =         [-0.226; -0.212; -0.148; -0.076; -0.006;  0.028;  0.064;  0.098;  0.132;  0.202;  0.238;  0.272;  0.334;  0.36;   0.38;  0.392;  0.394];
% Cd_interp    =         [ 0.392;  0.356;  0.264;  0.196;   0.22;    0.33;   0.493;  0.368;  0.330;  0.308;  0.300;  0.294;  0.286;  0.286;  0.297; 0.316;  0.344]; 

% NACA 2412
alpha_interp = deg2rad([   -20;    -15;    -10;     -5;      0;      5;      10;      15;      20]);
Cx_interp    =         [ 0.100;  0.058;  0.027;  0.010;  0.008;  0.022;   0.051;   0.093;   0.145];
Cy_interp    =         [-0.747; -0.550; -0.341; -0.120;  0.103;  0.325;   0.537;   0.740;   0.922];
Cmz_interp   =         [-0.204; -0.165; -0.122; -0.076; -0.029;  0.017;   0.063;   0.105;   0.143];
Cd_interp    =         [ 0.186;  0.154;  0.087; -0.203;   0.751; 0.422;   0.319;   0.288;   0.269];

% С массой 2.5 под крылом
% Cmz_interp   =         [-0.658; -0.415; -0.223; -0.089; -0.012; 0.002;  -0.045;  -0.151;  -0.313];

Cx  = interp1(alpha_interp, Cx_interp,   alpha, 'linear', 'extrap'); % Коэффициент силы сопротивления, зависит от угла атаки
Cy  = interp1(alpha_interp, Cy_interp,   alpha, 'linear', 'extrap'); % Коэффициент подъемной силы, зависит от угла атаки
Cmz = interp1(alpha_interp, Cmz_interp,  alpha, 'linear', 'extrap')*alpha; 
Cd  = interp1(alpha_interp, Cd_interp,   alpha, 'linear', 'extrap'); % Центр давления от передней кромки крыла

% % Коэфициенты из XFLR5 для крыла 2х1 м Профиль: NACA 2412
% Cx0 =  0.099229;   Cxa = -0.044517; 
% Cy0 =  0.0020099;  Cya =  2.6649;
% Cm0 = -0.0042205;  Cma =  0.58051;
% 
% Cx = getCx(alpha, Cx0, Cy0, Cya, l_wing, S)
% % Cx  = Cx0 + Cxa*alpha;
% Cy  = Cy0 + Cya*alpha;
% Cmz = Cm0 + Cma*alpha



Ra_wing     = [0.202-Cd 2 0]; % точка приложения силы в ССК, зависит от угла атаки

Fa_pssk = [-Cx Cy 0]*q*S;             % вектор аэродинамической силы в ПССК
Fa_ssk = Fa_pssk*pssk2ssk(alpha, 0);  % перевод в ССК

Fax = Fa_ssk(1);
Fay = Fa_ssk(2);

% Момент
% Ma_pssk = [0 0 Cmz*q*ba*S];
Ma  = cross(Ra_wing, Fa_ssk);
% Ma_ssk = Ma_pssk*pssk2ssk(alpha, 0);
Maz = Ma(3);
% [Mz, Mz*q*ba*S, Ma(3)]

% Maz = 0.02 - 0.4*alpha;

% Ma_ssk = Ma_pssk*pssk2ssk(alpha, 0);
% Maz = Ma_pssk(3);

% -------------------------------------------------------------------------
% Сила тяжести
% -------------------------------------------------------------------------
Fg_nsk = [0 -mass*g 0];
Fg_ssk = Fg_nsk*nsk2ssk(0, 0, theta);

Fgx = Fg_ssk(1);
Fgy = Fg_ssk(2);

% -------------------------------------------------------------------------
% Сила тяги двигателя
% -------------------------------------------------------------------------
k1 = 80.1;
k2 = 5.1;
k3 = 0.001;

k1 = 100.0;
k2 = 5.0;
k3 = 0.01;
Fpx = (Jzz/ly)*(Maz/Jzz*0 + (ry - 110)*((k1*k2*k3)/Vx + k1/Vx +k3*Vx) + ... 
             (theta + Vy/Vx)*(1 + Vx^2 + k1*k2 + k2*k3 + k1*k3) + ...
             omega_z*(k1 + k2 + k3));
         
P_MAX = 4;
if (Fpx < 0)     Fpx = 0; end;
if (Fpx > P_MAX) Fpx = P_MAX; end;

% Fpx = 1.5;
Mpz = -ly*Fpx;

gu(gi) = Fpx;
gt(gi) = t;
gi = gi + 1;

% -------------------------------------------------------------------------
% Уравнения движения
% -------------------------------------------------------------------------
rx_dot      = Vx*cos(theta) - Vy*sin(theta);
ry_dot      = Vx*sin(theta) + Vy*cos(theta);
Vx_dot      = ( omega_z*Vy +Fax + Fgx + Fpx) / mass;
Vy_dot      = (-omega_z*Vx +Fay + Fgy      ) / mass;
theta_dot   = omega_z;
omega_z_dot = (Maz + Mpz) / Jzz;
u_dot       = (0.1*Fpx - 10*u)/0.01;

dx_dt = [rx_dot; ry_dot; Vx_dot; Vy_dot; theta_dot; omega_z_dot; u_dot];

disp([num2str(t, '%10.2f:'), char(9), ...
      'Fpx:',   char(9), num2str([Fpx, Mpz],     '%10.2f'), char(9), char(9), ...
      'alpha:', char(9), num2str(rad2deg(alpha), '%10.2f'), char(9), ...
      'theta:', char(9), num2str(rad2deg(theta), '%10.2f'), char(9), ...
      'Fg:',    char(9), num2str([Fgx, Fgy],     '%10.2f'), char(9), ...
      'Fa:',    char(9), num2str([Fax, Fay],     '%10.2f'), char(9), ...
      'Maz:',   char(9), num2str(Maz,            '%10.2f'), char(9), char(9), ...
      'V:',     char(9), num2str([Vx, Vy],       '%10.2f'), char(9), ...
      'r:',     char(9), num2str([rx, ry],       '%10.2f'), char(9), char(9)] ...
  );

end

function Cx = getCx(alpha, Cx0, Cy0, Cy_alpha, l_wing, S)
    % see https://www.grc.nasa.gov/www/k-12/airplane/dragco.html
    % ba - хорда крыла
    % S - площадь крыла
    e  = 0.8; % Коэффициент эффективности Освальда, от 0,8 до 1,0
    AR = l_wing^2/S;
    Cx_induction = deg2rad(-2) + (Cy0 + Cy_alpha*alpha)^2 / (pi*e*AR);
    Cx = Cx0 + Cx_induction;
end

function plotCxCyCd()
    alpha = deg2rad(-20:0.1:20);
    alpha_interp = deg2rad([   -20;    -15;    -10;     -5;      0;      5;      10;      15;      20]);
    Cx_interp    =         [ 0.100;  0.058;  0.027;  0.010;  0.008;  0.022;   0.051;   0.093;   0.145];
    Cy_interp    =         [-0.747; -0.550; -0.341; -0.120;  0.103;  0.325;   0.537;   0.740;   0.922];
    Cmz_interp   =         [-0.204; -0.165; -0.122; -0.076; -0.029;  0.017;   0.063;   0.105;   0.143];
    Cd_interp    =         [ 0.186;  0.154;  0.087; -0.203;   0.751; 0.422;   0.319;   0.288;   0.269];

    % С массой 2.5 под крылом
    % Cmz_interp   =         [-0.658; -0.415; -0.223; -0.089; -0.012; 0.002;  -0.045;  -0.151;  -0.313];

    Cx  = interp1(alpha_interp, Cx_interp,   alpha, 'spline', 'extrap'); % Коэффициент силы сопротивления, зависит от угла атаки
    Cy  = interp1(alpha_interp, Cy_interp,   alpha, 'spline', 'extrap'); % Коэффициент подъемной силы, зависит от угла атаки
    Cmz = interp1(alpha_interp, Cmz_interp,  alpha, 'spline', 'extrap'); 
    Cd  = interp1(alpha_interp, Cd_interp,   alpha, 'cubic',  'extrap'); % Центр давления от передней кромки крыла

    plot_gost(rad2deg(alpha), Cx, '\alpha,\it  град', '\it C_x',         'fig_Cx_alpha');
    plot_gost(rad2deg(alpha), Cy, '\alpha,\it  град', '\it C_y',         'fig_Cy_alpha');
    plot_gost(rad2deg(alpha), Cd, '\alpha,\it  град', '\it x_{цд} ,  м', 'fig_xcd_alpha');

end

function plotCxCyCmz()

    Cx0 =  0.099229;   Cxa = -0.044517;
    Cy0 =  0.1020099;  Cya =  2.6649;
    Cm0 = -0.0042205;  Cma =  0.58051;
    figure(2)
    hold on;
    grid on;
    for alpha = deg2rad(-20):deg2rad(1):deg2rad(20)
        Cx = getCx(alpha, Cx0, Cy0, Cya, 1, 2)
        % Cx  = Cx0 + Cxa*alpha;
        Cy  = Cy0 + Cya*alpha;
        Cmz = Cm0 + Cma*alpha;

        plot(rad2deg(alpha), Cx, 'o');
        plot(rad2deg(alpha), Cy, 's');
        plot(rad2deg(alpha), Cmz, '*');
    end
end
