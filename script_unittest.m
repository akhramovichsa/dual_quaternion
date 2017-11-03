clc; clear all;

tolerance = 0.00000000001;

% check: dq_from_euler_translation. angle -> quaternion
display('CHECK: dq_from_euler_translation');
angle       = deg2rad([60, 60, 60]);
translation = [10.0 20.0 30.0];
q = angle2quat(angle(1), angle(2), angle(3), 'yzx');
dq = dq_from_euler_translation(angle, translation);

assert(all(q - dq(1:4) < tolerance), 'FAILED: dq_from_euler_translation')

% check: dq_get_translation_vector
display('CHECK: dq_get_translation_vector');
tr = dq_get_translation_vector(dq);

assert(all(translation - tr < tolerance), 'FAILED: dq_get_translation_vector');

% check: dq_get_rotation_euler
display('CHECK: dq_get_rotation_euler');
[angle_gamma, angle_psi, angle_theta] = dq_get_rotation_euler(dq);

assert(all(angle - rad2deg([angle_gamma, angle_psi, angle_theta]) < tolerance), 'FAILED: dq_get_rotation_euler');

% check dq_conj
display('CHECK: dq_conj');
dq = dq_from_euler_translation([0 0 0], [10 20 30]);
dq_conjug = dq_conj(dq);

assert(all(dq_get_translation_vector(dq) == -dq_get_translation_vector(dq_conjug)), 'FAILED: dq_conj');

%check: dq_inv
display('CHECK: dq_inv');
dq = dq_from_euler_translation(deg2rad([0 0 0]), [10 20 30])
dq_invert = dq_inv(dq)
dq_ones = dq_multiply(dq, dq_invert)

assert(all(dq_ones(2:8) < tolerance) && dq_ones(1) == 1, 'FAILED: dq_inv')

%check: dq_multiply
display('CHECK: dq_multiply');
angle_1       = deg2rad([0, 0, 0]);
translation_1 = [0 0 0];

angle_2       = deg2rad([0, 90, 0]);
translation_2 = [10 0 0];

dq1 = dq_from_euler_translation(angle_1, translation_1);
dq2 = dq_from_euler_translation(angle_2, translation_2);

dq_mul = dq_multiply(dq1, dq2)

tr = dq_get_translation_vector(dq_mul)
[r1, r2, r3] = dq_get_rotation_euler(dq_mul);
rad2deg([r1, r2, r3])

%check: dq_normalize ???????????
display('CHECK: dq_normalize');
dq1 = dq_from_euler_translation(deg2rad([0 0 0]), [100 0 0]);
dq2 = dq_from_euler_translation(deg2rad([150 0 0]), [0 0 0]);

dq = dq_multiply(dq1, dq2);
dq = dq_multiply(dq, dq2);

%%%%%%% dq_normalize(dq);
dq_get_translation_vector(dq);
[r1, r2, r3] = dq_get_rotation_euler(dq);
rad2deg([r1, r2, r3]);


% + dq_conj.m
% + dq_from_euler_translation.m
% + dq_from_quaternion_translation.m
% + dq_get_dual.m
% + dq_get_real.m
% + dq_get_rotation_euler.m
% + dq_get_rotation_quat.m
% + dq_get_translation_quat.m
% + dq_get_translation_vector.m
% dq_inv.m
% + dq_mod.m
% + dq_multiply.m
% + dq_norm.m
% - dq_normalize.m
% dq_transform_vector.m
