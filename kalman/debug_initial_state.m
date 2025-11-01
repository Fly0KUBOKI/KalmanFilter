% 初期状態の問題を調査
clear; clc;

addpath(genpath('ESKF'));
addpath(genpath('KF'));
addpath('GenerateData');

% データ読み込み
obs = read_csv('GenerateData/sensor_data.csv');
truth = readtable('GenerateData/truth_data.csv');

% ESKFの初期化
params = config_params();
dt = 0.0025;
kf = ESKF(obs, params.static_time, dt);

fprintf('=== 初期化直後（t=2.0秒、k=801） ===\n\n');

k = 801;  % t=2.0秒直後
euler = kf.getEuler();

fprintf('推定値:\n');
fprintf('  p = [%.6f, %.6f, %.6f]\n', kf.p(1), kf.p(2), kf.p(3));
fprintf('  v = [%.6f, %.6f, %.6f]\n', kf.v(1), kf.v(2), kf.v(3));
fprintf('  q = [%.6f, %.6f, %.6f, %.6f]\n', kf.q(1), kf.q(2), kf.q(3), kf.q(4));
fprintf('  euler = [%.6f, %.6f, %.6f]°\n', euler(1), euler(2), euler(3));
fprintf('  ba = [%.6f, %.6f, %.6f]\n', kf.ba(1), kf.ba(2), kf.ba(3));
fprintf('  bg = [%.6f, %.6f, %.6f]\n\n', kf.bg(1), kf.bg(2), kf.bg(3));

fprintf('真値:\n');
fprintf('  p = [%.6f, %.6f, %.6f]\n', truth.x(k), truth.y(k), truth.z(k));
fprintf('  v = [%.6f, %.6f, %.6f]\n', truth.vx(k), truth.vy(k), truth.vz(k));
fprintf('  euler = [%.6f, %.6f, %.6f]°\n\n', truth.roll(k), truth.pitch(k), truth.yaw(k));

fprintf('誤差:\n');
fprintf('  Δp = [%.6f, %.6f, %.6f]\n', kf.p(1)-truth.x(k), kf.p(2)-truth.y(k), kf.p(3)-truth.z(k));
fprintf('  Δv = [%.6f, %.6f, %.6f]\n', kf.v(1)-truth.vx(k), kf.v(2)-truth.vy(k), kf.v(3)-truth.vz(k));
fprintf('  Δeuler = [%.6f, %.6f, %.6f]°\n\n', euler(1)-truth.roll(k), euler(2)-truth.pitch(k), euler(3)-truth.yaw(k));

% センサデータを確認
fprintf('=== センサデータ（t=2.0秒） ===\n');
fprintf('加速度: [%.6f, %.6f, %.6f]\n', obs.ax(k), obs.ay(k), obs.az(k));
fprintf('角速度: [%.6f, %.6f, %.6f] rad/s\n', obs.wx(k), obs.wy(k), obs.wz(k));
fprintf('角速度: [%.6f, %.6f, %.6f] deg/s\n\n', rad2deg(obs.wx(k)), rad2deg(obs.wy(k)), rad2deg(obs.wz(k)));

% 1ステップ予測してみる
fprintf('=== 1ステップ予測テスト（k=801→802） ===\n');
a_meas = [obs.ax(k+1); obs.ay(k+1); obs.az(k+1)];
w_meas = deg2rad([obs.wx(k+1); obs.wy(k+1); obs.wz(k+1)]);

fprintf('次のステップのセンサ値:\n');
fprintf('  a = [%.6f, %.6f, %.6f]\n', a_meas(1), a_meas(2), a_meas(3));
fprintf('  w = [%.6f, %.6f, %.6f] rad/s\n', w_meas(1), w_meas(2), w_meas(3));
fprintf('  w = [%.6f, %.6f, %.6f] deg/s\n\n', rad2deg(w_meas(1)), rad2deg(w_meas(2)), rad2deg(w_meas(3)));

% バイアス補正後の値
a_corrected = a_meas - kf.ba;
w_corrected = w_meas - kf.bg;

fprintf('バイアス補正後:\n');
fprintf('  a_corr = [%.6f, %.6f, %.6f]\n', a_corrected(1), a_corrected(2), a_corrected(3));
fprintf('  w_corr = [%.6f, %.6f, %.6f] rad/s\n', w_corrected(1), w_corrected(2), w_corrected(3));
fprintf('  w_corr = [%.6f, %.6f, %.6f] deg/s\n\n', rad2deg(w_corrected(1)), rad2deg(w_corrected(2)), rad2deg(w_corrected(3)));

% ワールド座標系の真の加速度
Rb = quat_lib('quat_to_rotm', kf.q);
a_world = Rb * a_corrected;
g = kf.g;
a_true = a_world + g;

fprintf('ワールド座標系の真の加速度:\n');
fprintf('  a_world = [%.6f, %.6f, %.6f] (比力)\n', a_world(1), a_world(2), a_world(3));
fprintf('  g = [%.6f, %.6f, %.6f]\n', g(1), g(2), g(3));
fprintf('  a_true = a_world + g = [%.6f, %.6f, %.6f]\n\n', a_true(1), a_true(2), a_true(3));

% 真値の加速度と比較
fprintf('真値の加速度（数値微分で計算）:\n');
if k+2 <= length(truth.vx)
    a_true_vx = (truth.vx(k+2) - truth.vx(k)) / (2*dt);
    a_true_vy = (truth.vy(k+2) - truth.vy(k)) / (2*dt);
    a_true_vz = (truth.vz(k+2) - truth.vz(k)) / (2*dt);
    fprintf('  a_true_calc = [%.6f, %.6f, %.6f]\n', a_true_vx, a_true_vy, a_true_vz);
    fprintf('  誤差 = [%.6f, %.6f, %.6f]\n', a_true(1)-a_true_vx, a_true(2)-a_true_vy, a_true(3)-a_true_vz);
end
