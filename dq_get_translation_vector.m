function vout = dq_get_translation_vector(dq)
% DUALQUAT_GET_TRANSLATION_VECTOR Return translation vector from dual quaternion.

q_real = dq_get_real(dq);
q_dual = dq_get_dual(dq);

q_out = quatmultiply((2.0*q_dual'), quatconj(q_real'));

vout = q_out(2:4)'; % [x; y; z]
