function dq_out = dq_inv(dq)
% DUALQUAT_INV Calculate the inverse of a dual quaternion.

dq_out = dq_conj(dq)/dq_norm(dq);