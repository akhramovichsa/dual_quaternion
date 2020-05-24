% Манипулятор из 2-х звеньев
% Все плечи расположены вдоль оси X каждого шарнира
% Реализован метод FABRIK

clc; clear all;

eps = 0.01;  % Точность

q_one = [1; 0; 0; 0;  0; 0; 0; 0;];

% Шарнир в столе, не имеет плеча, может вращаться от 0 до 180 градусов вокруг оси Z
q0 = dq_from_euler_translation(deg2rad([0; 0; 0]), [0; 0; 0]);
d0 = 1;
qb = q0; % Базовая точка

% Плечо первого звена с шарниром на конце, может вращаться от 0 до 180 градусов вокруг оси Z
d1 = 1; % Длина плеча
q1 = dq_from_euler_translation(deg2rad([0; 0; 0]), [d1; 0; 0]);


% Плечо второго звена без шарнира на конце, не вращается
d2 = 1; % Длина плеча
q2 = dq_from_euler_translation(deg2rad([0; 0; 0]), [d2; 0; 0]);

% Начальное положение: q0*q1*q2
q_start = dq_multiply(q0, q1, q2);

[angle_gamma, angle_psi, angle_theta] = dq_get_rotation_euler(q_start);
dq_get_translation_vector(q_start);

% Конечное положение
q_finish = dq_from_euler_translation(deg2rad([0; 0; 0]), [0; 1.0; 0]);

figure(1)
clf
x_max = 3;
y_max = 3;
axis([-x_max x_max -y_max y_max])
hold on
grid on

q_current = q_start;
q_delta_global = dq_multiply(dq_conj(q_current), q_finish);
i = 1;       % Счетик шагов
i_max = 30; % Максимальное количество шагов
% while (norm(dq_get_translation_vector(q_delta_global)) > eps)
while i < i_max
    
    % ---------------------------------------------------------------------
    % FORWARD
    % ---------------------------------------------------------------------
    q2 = q_finish;
    
    % n-1 = 1
    %%r1      = norm(dq_get_translation_vector(dq_multiply(dq_conj(q1), q2)));
    %%lambda1 = d1/r1;
    %%q1      = dq_multiply((1 - lambda1)*q2, lambda1*q1);
    
    % n-2 = 0
    %%r0      = norm(dq_get_translation_vector(dq_multiply(dq_conj(q0), q1)));
    %%lambda0 = d0/r0;
    %%q0      = dq_multiply((1 - lambda0)*q1, lambda0*q0);
    
   % r0  = dq_multiply(dq_conj(q0), q1);
    % lambda0 = 0.1;
    ql1 = dq_from_euler_translation(deg2rad([0; 10; 0]), [0; 0; 0]);   % Шаг вращения вокруг оси Z
    % q0  = dq_multiply( dq_multiply(q1, (q_one - ql0)), dq_multiply(q0, ql0))
    q1  = dq_multiply((q_one - ql1), q2) + dq_multiply(ql1, q1);

%     ql0 = dq_from_euler_translation(deg2rad([0; 10; 0]), [0; 0; 0]);   % Шаг вращения вокруг оси Z
%     % q0  = dq_multiply( dq_multiply(q1, (q_one - ql0)), dq_multiply(q0, ql0))
%     q0  = dq_multiply((q_one - ql0), q1) + dq_multiply(ql0, q0);

    % ---------------------------------------------------------------------
    % BACKWARD
    % ---------------------------------------------------------------------
    q0 = qb;
    
    % n-2 = 0
    %%r0      = norm(dq_get_translation_vector(dq_multiply(dq_conj(q0), q1)));
    %%lambda0 = d0/r0;
    %%q0      = dq_multiply((1 - lambda0)*q1, lambda0*q0);
    
    % n-1 = 0
    %%r1      = norm(dq_get_translation_vector(dq_multiply(dq_conj(q1), q2)));
    %%lambda1 = d1/r1;
    %%q1      = dq_multiply((1 - lambda1)*q2, lambda1*q1);
   
    ql0 = dq_from_euler_translation(deg2rad([0; 10; 0]), [0; 0; 0]);   % Шаг вращения вокруг оси Z
    % q0  = dq_multiply( dq_multiply(q1, (q_one - ql0)), dq_multiply(q0, ql0))
    q1  = dq_multiply((q_one - ql0), q0) + dq_multiply(ql0, q1);

%     ql1 = dq_from_euler_translation(deg2rad([0; 10; 0]), [0; 0; 0]);   % Шаг вращения вокруг оси Z
%     % q0  = dq_multiply( dq_multiply(q1, (q_one - ql0)), dq_multiply(q0, ql0))
%     q2  = dq_multiply((q_one - ql1), q1) + dq_multiply(ql1, q2);

    % ---------------------------------------------------------------------
    % Рисуем анимацию
    % ---------------------------------------------------------------------
    % Звено 1
    q_plot = dq_multiply(q0, q1);
    q_plot_xyz_1 = dq_get_translation_vector(q_plot);
    plot(0, 0, 'o');
    plot(q_plot_xyz_1(1), q_plot_xyz_1(2), 'o');
    line([0 q_plot_xyz_1(1)], [0 q_plot_xyz_1(2)]);
    
    % Звено 2
    q_plot = dq_multiply(q_plot, q2);
    q_plot_xyz_2 = dq_get_translation_vector(q_plot);
    plot(q_plot_xyz_2(1), q_plot_xyz_2(2), 's');
    line([q_plot_xyz_1(1) q_plot_xyz_2(1)], [q_plot_xyz_1(2) q_plot_xyz_2(2)]);
    
    pause(0.1);
    
    i = i + 1;

end