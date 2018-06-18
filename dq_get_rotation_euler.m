function [angle_y, angle_z, angle_x] = dq_get_rotation_euler(dq)
% DUALQUAT_GET_ROTATION Return rotation quaterion from dual quaternion.

[angle_y, angle_z, angle_x] = quat2angle(dq(1:4)', 'yzx');
