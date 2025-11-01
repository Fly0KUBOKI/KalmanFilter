% 発散の原因を診断
clear; clc;

addpath(genpath('ESKF'));
addpath(genpath('KF'));
addpath('GenerateData');

% データ読み込み
obs = read_csv('GenerateData/sensor_data.csv');
truth = readtable('GenerateData/truth_data.csv');

% 短期間テスト（10秒、4000サンプル）
N_test = 4000;
dt = 0.0025;

fprintf('=== 短期テスト（10秒） ===\n\n');

% ESKFの初期化（標準設定）
params = config_params();
kf = ESKF(obs, params.static_time, dt);

fprintf('初期状態:\n');
fprintf('  ba = [%.6f, %.6f, %.6f]\n', kf.ba(1), kf.ba(2), kf.ba(3));
fprintf('  bg = [%.6f, %.6f, %.6f]\n', kf.bg(1), kf.bg(2), kf.bg(3));
fprintf('  q  = [%.6f, %.6f, %.6f, %.6f]\n\n', kf.q(1), kf.q(2), kf.q(3), kf.q(4));

% 静止期間終了直後から開始（k=801）
k_start = 801;
k_samples = [k_start, k_start+100, k_start+500, k_start+1000, k_start+2000];

fprintf('時刻\t\tpx\t\tpy\t\tvx\t\tvy\t\troll\t\tpitch\t\tyaw\n');
fprintf('------------------------------------------------------------\n');

for i = 1:length(k_samples)
    k = k_samples(i);
    if k > k_start
        % k_startからkまで積分
        if i == 1
            j_start = k_start;
        else
            j_start = k_samples(i-1) + 1;
        end
        for j = j_start : k
            kf.updateFilter(obs, j);
        end
    end
    
    t = obs.time(k);
    euler = kf.getEuler();
    
    fprintf('%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n', ...
        t, kf.p(1), kf.p(2), kf.v(1), kf.v(2), ...
        euler(1), euler(2), euler(3));
end

fprintf('\n=== 真値との比較（t=%.4f秒） ===\n', obs.time(k_samples(end)));
k_last = k_samples(end);
fprintf('推定値:\n');
fprintf('  p = [%.4f, %.4f, %.4f]\n', kf.p(1), kf.p(2), kf.p(3));
fprintf('  v = [%.4f, %.4f, %.4f]\n', kf.v(1), kf.v(2), kf.v(3));
euler = kf.getEuler();
fprintf('  euler = [%.4f, %.4f, %.4f]°\n\n', euler(1), euler(2), euler(3));

fprintf('真値:\n');
fprintf('  p = [%.4f, %.4f, %.4f]\n', truth.x(k_last), truth.y(k_last), truth.z(k_last));
fprintf('  v = [%.4f, %.4f, %.4f]\n', truth.vx(k_last), truth.vy(k_last), truth.vz(k_last));
fprintf('  euler = [%.4f, %.4f, %.4f]°\n\n', truth.roll(k_last), truth.pitch(k_last), truth.yaw(k_last));

fprintf('誤差:\n');
fprintf('  Δp = [%.4f, %.4f, %.4f]\n', kf.p(1)-truth.x(k_last), kf.p(2)-truth.y(k_last), kf.p(3)-truth.z(k_last));
fprintf('  Δv = [%.4f, %.4f, %.4f]\n', kf.v(1)-truth.vx(k_last), kf.v(2)-truth.vy(k_last), kf.v(3)-truth.vz(k_last));
fprintf('  Δeuler = [%.4f, %.4f, %.4f]°\n', euler(1)-truth.roll(k_last), euler(2)-truth.pitch(k_last), euler(3)-truth.yaw(k_last));

% 発散の開始点を特定
fprintf('\n=== 発散の進行を確認 ===\n');
fprintf('時刻\t\t位置誤差\t速度誤差\t姿勢誤差\n');
fprintf('------------------------------------------------------------\n');

for i = 1:length(k_samples)
    k = k_samples(i);
    t = obs.time(k);
    
    % 真値との誤差
    pos_err = norm([kf.p(1)-truth.x(k), kf.p(2)-truth.y(k), kf.p(3)-truth.z(k)]);
    vel_err = norm([kf.v(1)-truth.vx(k), kf.v(2)-truth.vy(k), kf.v(3)-truth.vz(k)]);
    euler = kf.getEuler();
    att_err = norm([euler(1)-truth.roll(k), euler(2)-truth.pitch(k), euler(3)-truth.yaw(k)]);
    
    fprintf('%.4f\t%.6f\t%.6f\t%.6f\n', t, pos_err, vel_err, att_err);
end
