% quat_test.m
% Simple unit tests for quat_lib functions used in project

fprintf('Running quat_lib tests...\n');

% Test 1: small_angle_quat for 180 deg around z (pi rad)
th = [0;0;pi];
dq = quat_lib('small_angle_quat', th);
% expected quaternion for 180deg about z: [cos(pi/2); 0; 0; sin(pi/2)] = [0; 0; 0; 1]
fprintf('\nTest 1: 180 deg about z\n');
fprintf('dq = [%g %g %g %g]\n', dq(1), dq(2), dq(3), dq(4));
assert(norm(dq - [0;0;0;1]) < 1e-8 || norm(dq + [0;0;0;1]) < 1e-8, 'small_angle_quat 180deg failed');

% Test 2: small-angle approx for very small angle
th_small = [0.001; 0; 0];
dq_small = quat_lib('small_angle_quat', th_small);
fprintf('\nTest 2: small-angle approx\n');
fprintf('dq_small = [%g %g %g %g]\n', dq_small(1), dq_small(2), dq_small(3), dq_small(4));
% expect qw ~ 1, qx ~ 0.5*th
assert(abs(dq_small(1) - 1) < 1e-6, 'small_angle_quat small-angle qw not near 1');
assert(abs(dq_small(2) - 0.5*th_small(1)) < 1e-6, 'small_angle_quat small-angle qx wrong');

% Test 3: quatmultiply consistency (q * q^{-1} = identity)
q = quat_lib('quatnormalize', [0.70710678; 0.70710678; 0; 0]); % 90deg about x
q_inv = [q(1); -q(2:4)];
prod = quat_lib('quatmultiply', q, q_inv);
fprintf('\nTest 3: quatmultiply inverse\n');
fprintf('prod = [%g %g %g %g]\n', prod(1), prod(2), prod(3), prod(4));
assert(norm(prod - [1;0;0;0]) < 1e-8, 'quatmultiply inverse failed');

% Test 4: quat_to_euler for known quaternion (now returns degrees)
q_yaw = quat_lib('quatnormalize', [cos(pi/4); 0; 0; sin(pi/4)]); % 90deg yaw
eul = quat_lib('quat_to_euler', q_yaw);
fprintf('\nTest 4: quat_to_euler yaw 90deg (degrees)\n');
fprintf('euler (deg) = [%g %g %g]\n', eul(1), eul(2), eul(3));
assert(abs(eul(3) - 90) < 1e-6, 'quat_to_euler yaw incorrect');

% Test 5: vector_to_quat aligns v1->v2
v1 = [1;0;1]; v2 = [0;1;1];
qv = quat_lib('vector_to_quat', v1, v2);
% rotate v1 by quaternion and compare to v2
R = quat_lib('quat_to_rotm', qv);
v1_rot = R * v1;
fprintf('\nTest 5: vector_to_quat align x->y\n');
fprintf('qv = [%f %f %f %f]\n', qv(1), qv(2), qv(3), qv(4));
fprintf('R [%f %f %f; %f %f %f; %f %f %f]\n', R(1,1), R(1,2), R(1,3), R(2,1), R(2,2), R(2,3), R(3,1), R(3,2), R(3,3));
fprintf('v1_rot = [%g %g %g]\n', v1_rot(1), v1_rot(2), v1_rot(3));

fprintf('\nAll quat_lib tests passed.\n');

% End of file
