function [angle_x, angle_y, angle_z] = dq_get_rotation_euler(dq)
% DUALQUAT_GET_ROTATION Return rotation quaterion from dual quaternion.

[angle_x, angle_y, angle_z] = quat2angle(dq(1:4), 'xyz');
