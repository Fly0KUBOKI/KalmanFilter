% test_ukf_n6_m3.m - UKF n=6, m=3 テスト

clc;
clear all;

% パス追加
proj_root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(proj_root, 'cpp')));

fprintf('=== UKF n=6, m=3 Direct MEX Test ===\n\n');

% テストデータ準備
x = [1; 2; 3; 0.1; 0.2; 0.3];  % 6x1 (位置+速度)
P = eye(6) * 0.1;                % 6x6
z = [1.05; 2.05; 3.05];          % 3x1 (位置観測)
h_func = @(x_pv) x_pv(1:3);      % 観測関数：位置のみ
R = eye(3) * 0.01;               % 3x3
alpha = 1e-3;
beta = 2;
kappa = 0;

fprintf('Input dimensions:\n');
fprintf('  x: %dx%d\n', size(x,1), size(x,2));
fprintf('  P: %dx%d\n', size(P,1), size(P,2));
fprintf('  z: %dx%d\n', size(z,1), size(z,2));
fprintf('  R: %dx%d\n\n', size(R,1), size(R,2));

% MEXファイルの存在確認
mex_path = which('mex_ukf_update');
if isempty(mex_path)
    error('mex_ukf_update not found in path');
end
fprintf('Using MEX: %s\n', mex_path);

% MEXファイル情報
mex_info = dir(mex_path);
fprintf('MEX timestamp: %s\n\n', mex_info.date);

% MEXを直接呼び出し
fprintf('Calling mex_ukf_update directly...\n');
try
    [x_upd, P_upd, K, S, y] = mex_ukf_update(x, P, z, h_func, R, alpha, beta, kappa);
    fprintf('SUCCESS!\n');
    fprintf('  x_upd: %dx%d\n', size(x_upd,1), size(x_upd,2));
    fprintf('  P_upd: %dx%d\n', size(P_upd,1), size(P_upd,2));
    fprintf('  K: %dx%d\n', size(K,1), size(K,2));
    fprintf('  S: %dx%d\n', size(S,1), size(S,2));
    fprintf('  y: %dx%d\n\n', size(y,1), size(y,2));
    fprintf('  Innovation y: [%.4f, %.4f, %.4f]\n', y(1), y(2), y(3));
catch ME
    fprintf('FAILED!\n');
    fprintf('  Error: %s\n', ME.message);
    fprintf('  Identifier: %s\n\n', ME.identifier);
    rethrow(ME);
end

fprintf('\n=== Test Complete ===\n');
