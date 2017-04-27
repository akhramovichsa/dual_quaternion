function vector_out = dq_transform_vector(pos, dq)
%dualquat_transform_vector Transform and roatate dual vector

% not working
% dq1 = dualquat_multiply(dq, dual_vector);
% dual_vector_out = dualquat_multiply(dq1, dualquat_conj(dq));

% working
dual_vector = [0 0 0 0  0 pos];
dq1 = dq_multiply(dq_conj(dq), dual_vector);
dual_vector_out = dq_multiply(dq1, dq);
vector_out = dual_vector_out(6:8);

% working
% see http://dev.theomader.com/dual-quaternion-skinning/
% real_xyz = dq(2:4);
% real_w   = dq(1);
% dual_xyz = dq(6:8);
% dual_w   = dq(5);
% 
% vector_out    = pos + ...
%                 2 * cross( real_xyz, cross(real_xyz, pos) + real_w*pos ) + ...
%                 2 * (real_w * dual_xyz - dual_w * real_xyz + ...
%                     cross( real_xyz, dual_xyz));
end