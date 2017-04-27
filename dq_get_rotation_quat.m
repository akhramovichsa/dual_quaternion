function qout = dq_get_rotation_quat(dq)
% DUALQUAT_GET_ROTATION Return rotation quaterion from dual quaternion.

qout = dualquat_get_real(dq);
