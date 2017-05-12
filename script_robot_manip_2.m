% ћанипул€тор из 2-х звеньев
% ¬се плечи расположены вдоль оси X каждого шарнира

clc; clear all;

% Ўарнир в столе, не имеет плеча, может вращатьс€ от 0 до 180 градусов вокруг оси Z
q0 = dq_from_euler_translation(deg2rad([0, 0, 0]), [0 0 0])

% ѕлечо первого звена с шарниром на конце, может вращатьс€ от 0 до 180 градусов вокруг оси Z
q1 = dq_from_euler_translation(deg2rad([0, 0, 0]), [1 0 0])

% ѕлечо второго звена без шарнира на конце, не вращаетс€
q2 = dq_from_euler_translation(deg2rad([0, 0, 0]), [1 0 0])


% Ќачальное положение: q0*q1*q2
q_start = dq_multiply(q0, q1, q2);

[angle_gamma, angle_psi, angle_theta] = dq_get_rotation_euler(q_start)
dq_get_translation_vector(q_start)


%  онечное положение
q_finish = dq_from_euler_translation(deg2rad([0, 0, 0]), [0 1.5 0])

% ћетод покоординатного спуска
q_h_z = dq_from_euler_translation(deg2rad([0, 0, 0.2]), [0 0 0]);   % Ўаг вращени€ вокруг оси Z
eps = 0.005;  % “очность
i = 1;    % —четик шагов
i_max = 700; % ћаксимальное количество шагов

figure(1)
clf
x_max = 3;
y_max = 3;
axis([-x_max x_max -y_max y_max])
hold on
grid on

while i < i_max
    % ---------------------------------------------------------------------
    % —пуск по первой координате в положительном направлении: q0*q_h_z*q1*q2
    % ---------------------------------------------------------------------
    q_cur_1_plus = dq_multiply(q0, q_h_z, q1, q2);
    
    % —пуск по первой координате в отрицательном направлении
    q_cur_1_minus = dq_multiply(q0, dq_conj(q_h_z), q1, q2);
    
    % [angle_gamma, angle_psi, angle_theta] = dq_get_rotation_euler(q_cur_1_plus);
    % dq_get_translation_vector(q_cur_1_plus);

    % [angle_gamma, angle_psi, angle_theta] = dq_get_rotation_euler(q_cur_1_minus)
    % dq_get_translation_vector(q_cur_1_minus)

    % —равнение, выбор минимального
    q_delta_plus     = dq_multiply(dq_conj(q_cur_1_plus), q_finish); % –асчет разницы между текущим и конечным положением
    delta_norm_plus  = norm(dq_get_translation_vector(q_delta_plus)); % рассто€ние до конечной точки

    q_delta_minus    = dq_multiply(dq_conj(q_cur_1_minus), q_finish);
    delta_norm_minus = norm(dq_get_translation_vector(q_delta_minus)); % рассто€ние до конечной точки
    
    if (delta_norm_plus > delta_norm_minus)
        q0 = dq_multiply(q0, dq_conj(q_h_z));
    else
        q0 = dq_multiply(q0, q_h_z);
    end
    
    % [angle_gamma, angle_psi, angle_theta] = dq_get_rotation_euler(q_delta)
    % dq_get_translation_vector(q_delta)
    

    % ---------------------------------------------------------------------
    % —пуск по второй координате в положительном направлении: q0*q1*q_h_z*q2
    % ---------------------------------------------------------------------
    q_cur_2_plus = dq_multiply(q0, q1, q_h_z, q2);
    
    % —пуск по первой координате в отрицательном направлении
    q_cur_2_minus = dq_multiply(q0, q1, dq_conj(q_h_z), q2);
    
    % [angle_gamma, angle_psi, angle_theta] = dq_get_rotation_euler(q_cur_1_plus);
    % dq_get_translation_vector(q_cur_1_plus);

    % [angle_gamma, angle_psi, angle_theta] = dq_get_rotation_euler(q_cur_1_minus)
    % dq_get_translation_vector(q_cur_1_minus)

    % —равнение, выбор минимального
    q_delta_plus     = dq_multiply(dq_conj(q_cur_2_plus), q_finish);
    delta_norm_plus  = norm(dq_get_translation_vector(q_delta_plus)); % рассто€ние до конечной точки

    q_delta_minus    = dq_multiply(dq_conj(q_cur_2_minus), q_finish);
    delta_norm_minus = norm(dq_get_translation_vector(q_delta_minus)); % рассто€ние до конечной точки
    
    if (delta_norm_plus > delta_norm_minus)
        % q1 = dq_multiply(q0, q1);
        q1 = dq_multiply(q1, dq_conj(q_h_z));
    else
        % q1 = dq_multiply(q0, q1);
        q1 = dq_multiply(q1, q_h_z);
    end
    
    % [angle_gamma, angle_psi, angle_theta] = dq_get_rotation_euler(q_delta)
    % dq_get_translation_vector(q_delta)

    
    % ---------------------------------------------------------------------
    % –исуем анимацию
    % ---------------------------------------------------------------------
    % «вено 1
    q_plot = dq_multiply(q0, q1);
    q_plot_xyz_1 = dq_get_translation_vector(q_plot);
    plot(0, 0, 'o');
    plot(q_plot_xyz_1(1), q_plot_xyz_1(2), 'o');
    line([0 q_plot_xyz_1(1)], [0 q_plot_xyz_1(2)]);
    
    % «вено 2
    q_plot = dq_multiply(q_plot, q2);
    q_plot_xyz_2 = dq_get_translation_vector(q_plot);
    plot(q_plot_xyz_2(1), q_plot_xyz_2(2), 's');
    line([q_plot_xyz_1(1) q_plot_xyz_2(1)], [q_plot_xyz_1(2) q_plot_xyz_2(2)]);
    
    pause(0.01);
    
    % ---------------------------------------------------------------------
    % √лобальное сравнение достижени€ финишной точки
    % ---------------------------------------------------------------------
    q_current = dq_multiply(q0, q1, q2);
    q_delta_global = dq_multiply(dq_conj(q_current), q_finish);
    
    if (norm(dq_get_translation_vector(q_delta_global)) < eps)
        break;
    end
    
    i = i + 1;
    
    [i norm(dq_get_translation_vector(q_delta_global))];
end
