function out = dq_mod(dq)
% DUALQUAT_MOD  Calculate the modulus of a dual quaternion.

q_real_mod = sqrt(dq(1)^2 + dq(2)^2 + dq(3)^2 + dq(4)^2);

out = [q_real_mod; ...
       (dq(1)*dq(5) + dq(2)*dq(6) + dq(3)*dq(7) + dq(4)*dq(8))/q_real_mod];
