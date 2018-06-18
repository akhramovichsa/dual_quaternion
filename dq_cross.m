function dq_out = dq_cross(dq1, dq2, varargin)
%dualquat_cross Dual quaternion cross product

% dq_out = 0.5 * (dualquat_multiply(dq1, dq2) + dualquat_multiply(dq2, dq1))

q1_real = dq_get_real(dq1);
q1_dual = dq_get_dual(dq1);

q2_real = dq_get_real(dq2);
q2_dual = dq_get_dual(dq2);

dq_out = [quatcross(q1_real, q2_real); quatcross(q1_real, q2_dual) + quatcross(q1_dual, q2_real)];

% next arguments
nVarargs = length(varargin);
for k = 1:nVarargs
    q1_real = dq_get_real(dq_out);
    q1_dual = dq_get_dual(dq_out);
    q2_real = dq_get_real(varargin{k});
    q2_dual = dq_get_dual(varargin{k});
    
    dq_out = [quatcross(q1_real, q2_real); quatcross(q1_real, q2_dual) + quatcross(q1_dual, q2_real)];
end

% dq_out = [quatcross(q1_real, q2_real), quatcross(q1_real, q2_dual)];

end

function q_out = quatcross(q, p)
% q = quatmultiply(q, p);
% q_out = [0 q(2) q(3) q(4)];

% q_out = 0.5*(quatmultiply(q, p) - quatmultiply(p, q));

q_out = [0; q(1)*p(2:4) + p(1)*q(2:4) + cross(q(2:4), p(2:4))];
end