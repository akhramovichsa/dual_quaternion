function dq_out = dq_multiply(dq1, dq2, varargin)
% DUALQUAT_MULTIPLY Calculate the product of biquaternions.
%
%   Examples: 
%   Determine the product of two 1-by-8 dual quaternions:
%      bq1 = [1; 0; 1; 0; 0; 0; 0; 0];
%      bq2 = [1; 0.5; 0.5; 0.75; 0; 0; 0; 0];
%      bq3 = dq_multiply(bq1, bq2) 
%
%   Note: Dual quaternion multiplication is not commutative. 

if (size(dq1,1) ~= 8)
    error(message('dualquat_multiply:wrongDimension1'));
end

if (size(dq2,1) ~= 8)
    error(message('dualquat_multiply:wrongDimension2'));
end

if (size(dq1,1) ~= size(dq2,1))
    error(message('dualquat_multiply:wrongDimension3'));
end

q1_real = dq_get_real(dq1);
q1_dual = dq_get_dual(dq1);
q2_real = dq_get_real(dq2);
q2_dual = dq_get_dual(dq2);

dq_out = [quatmultiply(q1_real', q2_real'), quatmultiply(q1_real', q2_dual') + quatmultiply(q1_dual', q2_real')]';

% next arguments
nVarargs = length(varargin);
for k = 1:nVarargs
    q1_real = dq_get_real(dq_out);
    q1_dual = dq_get_dual(dq_out);
    q2_real = dq_get_real(varargin{k});
    q2_dual = dq_get_dual(varargin{k});
    
    dq_out = [quatmultiply(q1_real', q2_real'), quatmultiply(q1_real', q2_dual') + quatmultiply(q1_dual', q2_real')]';
    
    % fprintf('   %d\n', varargin{k})
end

end