function dq_out = dq_inv(dq)
% DUALQUAT_INV Calculate the inverse of a dual quaternion.
% Челноков Ю.Н. стр 70
% Гордеев стр. 140

q_real_norm = dq(1)^2 + dq(2)^2 + dq(3)^2 + dq(4)^2;

% знак минус обязателен, см. литературу выше
dq_norm_inv = [q_real_norm; ...
               - (dq(1)*dq(5) + dq(2)*dq(6) + dq(3)*dq(7) + dq(4)*dq(8))/q_real_norm];

dq_c = dq_conj(dq);

dq_out = [dq_norm_inv(1) * dq_c(1:4); ...
          dq_norm_inv(1) * dq_c(5) + dq_norm_inv(2) * dq_c(1); ...
          dq_norm_inv(1) * dq_c(6) + dq_norm_inv(2) * dq_c(2); ...
          dq_norm_inv(1) * dq_c(7) + dq_norm_inv(2) * dq_c(3); ...
          dq_norm_inv(1) * dq_c(8) + dq_norm_inv(2) * dq_c(4)];