clc; clear all;

v0 = [0; 0; 0];
q01 = dq_from_euler_translation(deg2rad([0; 0; 0]), [0; 1; 0])
q12 = dq_from_euler_translation(deg2rad([0; 0; 0]), [1; 0; 0])

q1 = dq_multiply(q01, q12)

v1 = dq_transform_vector(v0, q1)

% [angle_gamma, angle_psi, angle_theta] = dq_get_rotation_euler(q1)
% dq_get_translation_vector(q1)

% 
% q12 = dq_from_euler_translation(deg2rad([0, 0, 0]), [1 0 0])
% q2 = dq_multiply(q1, q12)
%  
% [angle_gamma, angle_psi, angle_theta] = dq_get_rotation_euler(q2)
% dq_get_translation_vector(q2)
% 
% 
% q1 = dq_transform_vector(q2, [1 0 0 0  0 0 0 0])
% 
% [angle_gamma, angle_psi, angle_theta] = dq_get_rotation_euler(q1)
% dq_get_translation_vector(q1)
% 
% quatrotate(dq_get_real(q1), dq_get_translation_vector(q1))

