function dqout = dq_from_euler_translation(euler, translation)
% DUALQUAT_FROM_EULER_TRANSLATION Return rotation quaterion from dual quaternion.
% euler - vector 1x3 [roll, yaw, pitch] or [gamma, psi, theta], 
% angle aroud [OX OY OZ]
% translation - vector 1x3 [x, y, z]

dqout = zeros(1,8);
dqout(1) = 1;

% real part
q_real = angle2quat(euler(1), euler(2), euler(3), 'xyz'); 

% dual part
q_dual = quatmultiply([0 translation], q_real)/2;

dqout = [q_real q_dual];
