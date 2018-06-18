function out = dq_norm(dq)
% DUALQUAT_NORM Calculate the norm of a dual quaternion.

out = [dq(1)^2 + dq(2)^2 + dq(3)^2 + dq(4)^2; ...
       2*(dq(1)*dq(5) + dq(2)*dq(6) + dq(3)*dq(7) + dq(4)*dq(8))];
