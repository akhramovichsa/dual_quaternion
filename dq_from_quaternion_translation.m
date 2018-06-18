function dqout = dq_from_quaternion_translation(q, translation)
% DUALQUAT_FROM_QUATERNION_TRANSLATION Return rotation quaterion from dual quaternion.
% q - vector 1x4 [q0; qx; qy; qz] or [q0; q1; q2; q3]
% translation - vector 1x3 [x; y; z]

% dual part
q_dual = quatmultiply([0 translation'], q)/2.0;

dqout = [q'; q_dual'];
