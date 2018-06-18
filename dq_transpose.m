function dq_out = dq_transpose(dq)
%DUALQUAT_TRANSPOSE Dual quaternion tranpose

dq_out = [dq(5:8); dq(1:4)];
end

