function dqout = dq_conj(dq)
% DUALQUAT_CONJ Return conjugate dual quaternion.

dqout = [dq(1); -dq(2); -dq(3); -dq(4); ...
         dq(5); -dq(6); -dq(7); -dq(8)];
