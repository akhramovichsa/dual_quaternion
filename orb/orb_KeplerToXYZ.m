function rv = orb_KeplerToXYZ (a, e, i, OMEGA, u, MU)
% Перевод Кеплеровых элементов орбиты в декартовую геоцентрическую систему
% координат
%
% Входные данные: 
% шесть элементов орбиты, определяющих положение КА на орбите:
% a        - большая полуось          [км],
% e        - эксцентриситет,
% i        - наклонение               [рад],
% $\Omega$ - долгота восходящего узла [рад],
% u        - истинная аномалия        [рад].
% MU       - Гравитационный параметр планеты [км^3/с^2], для Земли = 398600.435608.
%
% Выходные данные:
% rv(6) = [rx; ry; rz, Vx; Vy; Vz] - Радиус-вектор положения КА на орбите
% [км] и вектор скорости КА [км/с].
%
% Example:
%
% >> orb_KeplerToXYZ(6378 + 122, 0.0, 0.0, 0.0, 0.0, 398600.435608)
% 
% ans =
% 
%                       6500
%                          0
%                          0
%                          0
%            7.8309095218686
%                          0

cos_u     = cos(u); 
sin_u     = sin(u);
cos_i     = cos(i);
sin_i     = sin(i);
cos_OMEGA = cos(OMEGA);
sin_OMEGA = sin(OMEGA);

p              = a*(1 - e*e);	     % Фокальный параметр, [км]
r              = p/(1 + e*cos_u);    % Расстояние до КА, [км]
V_radial       = sqrt(MU/p)*e*sin_u; % Радиальная скорость, [км/c]
V_angular_rate = sqrt(MU*p)/r^2;     % Угловая скокость, [рад/с]
    
% q = V_radial/r_orb;
% s = V_angular_rate*r_orb;
    
rx = r * (cos_u*cos_OMEGA - sin_u*sin_OMEGA*cos_i);
ry = r * (cos_u*sin_OMEGA + sin_u*sin_OMEGA*cos_i);
% ry = r * (cos_u*sin_OMEGA + sin_u*cos_OMEGA*cos_i);
rz = r * (sin_u*sin_i);

Vx = (V_radial/r) * rx + (V_angular_rate*r) * (- sin_u*cos_OMEGA - cos_u*sin_OMEGA*cos_i);
Vy = (V_radial/r) * ry + (V_angular_rate*r) * (- sin_u*sin_OMEGA + cos_u*cos_OMEGA*cos_i);
Vz = (V_radial/r) * rz + (V_angular_rate*r) * cos_u*sin_i;

rv = [rx; ry; rz; Vx; Vy; Vz];

end