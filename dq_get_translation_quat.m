function qout = dq_get_translation_quat(dq)
% DUALQUAT_GET_TRANSLATION_QUAT Return translation quaternion from dual quaternion.

qout = dualquat_get_dual(dq);
