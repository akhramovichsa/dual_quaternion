function dqout = dq_normalize(dq)
% DUALQUAT_NORMALIZE Normalize a dual quaternion.

assert(false, 'function not checked')
q_real_mod = quatmod(dualquat_get_real(dq));

dqout = dq / q_real_mod;
