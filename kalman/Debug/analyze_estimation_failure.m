function analyze_estimation_failure(sensor_file, truth_file, estimation_file, varargin)
% analyze_estimation_failure - 推定失敗の原因を総合的に分析（簡潔版）
%
% 使用法:
%   analyze_estimation_failure(sensor_file, truth_file, estimation_file)
%   analyze_estimation_failure(..., 'StaticTime', 2.0, 'dt', 0.0025, 'AnalysisLength', Inf, 'PlotResults', true, 'Verbose', false)
%
% 入力:
%   sensor_file      - センサーデータのCSVファイルパス
%   truth_file       - 真値データのCSVファイルパス
%   estimation_file  - 推定結果のCSVファイルパス
%
% オプションパラメータ:
%   'StaticTime'     - 静止期間[秒] (デフォルト: 2.0)
%   'dt'             - サンプリング周期[秒] (デフォルト: 0.0025)
%   'AnalysisLength' - 分析期間[秒] (デフォルト: Inf -> データ末尾まで)
%   'PlotResults'    - プロット表示 (デフォルト: true)
%   'Verbose'        - 詳細ログ表示 (デフォルト: false)
%
% 例:
%   % 失敗データを分析
%   analyze_estimation_failure(...
%       'GenerateData/sensor_data_f.csv', ...
%       'GenerateData/truth_data_f.csv', ...
%       'Results/estimation_f.csv');
%
%   % 成功データと比較
%   analyze_estimation_failure(...
%       'GenerateData/sensor_data.csv', ...
%       'GenerateData/truth_data.csv', ...
%       'Results/estimation.csv');

% デフォルトパラメータ
p = inputParser;
addRequired(p, 'sensor_file');
addRequired(p, 'truth_file');
addRequired(p, 'estimation_file');
addParameter(p, 'StaticTime', 2.0);
addParameter(p, 'dt', 0.0025);
addParameter(p, 'AnalysisLength', Inf);
addParameter(p, 'PlotResults', true);
addParameter(p, 'Verbose', false);
parse(p, sensor_file, truth_file, estimation_file, varargin{:});

static_time = p.Results.StaticTime;
dt = p.Results.dt;
analysis_length = p.Results.AnalysisLength;
plot_results = p.Results.PlotResults;
verbose = p.Results.Verbose;

% パスを追加
addpath('GenerateData');

% --- データ読み込み（簡潔化: エラーは呼び出し元で管理）
obs = read_csv(sensor_file);
truth = readtable(truth_file);
estimation = readtable(estimation_file);

if ~ismember('time', estimation.Properties.VariableNames)
    error('estimation table must contain ''time'' column');
end

function val = getFieldOrDefault(tbl, fieldName, defaultVal)
% getFieldOrDefault - table/struct のフィールドがあれば返す、なければ default
    if istable(tbl) && ismember(fieldName, tbl.Properties.VariableNames)
        val = tbl.(fieldName);
    elseif isstruct(tbl) && isfield(tbl, fieldName)
        val = tbl.(fieldName);
    else
        val = defaultVal;
    end
end

% dt の自動補正（ユーザ指定が無効な場合）
if isempty(dt) || ~isnumeric(dt) || dt <= 0
    dt = mean(diff(obs.time));
end

static_samples = max(0, floor(static_time / dt));

% 開始時刻は静止期間直後のサンプル時刻
t_start = obs.time(min(static_samples+1, length(obs.time)));

% 終了時刻は AnalysisLength またはデータ末尾の小さい方
% truth.time がベクトルで来る可能性があるため、末尾の時刻（スカラー）を取り出して比較する
t_truth = getFieldOrDefault(truth, 'time', estimation.time(end));
if isempty(t_truth)
    t_truth_end = estimation.time(end);
elseif isvector(t_truth)
    % 列ベクトルや行ベクトルどちらでも末尾要素を使う
    t_truth_end = t_truth(end);
else
    % 想定外の形状の場合はデフォルトを使う
    t_truth_end = estimation.time(end);
end

if isinf(analysis_length) || analysis_length <= 0
    t_end = min([obs.time(end), estimation.time(end), t_truth_end]);
else
    t_end = min(t_start + analysis_length, min([obs.time(end), estimation.time(end), t_truth_end]));
end

% インデックス範囲（estimation の時刻に揃える）
start_k = find(estimation.time >= t_start, 1, 'first');
end_k = find(estimation.time <= t_end, 1, 'last');
if isempty(start_k) || isempty(end_k) || end_k < start_k
    error('解析可能な時刻範囲がありません (t_start=%.3f, t_end=%.3f)', t_start, t_end);
end

analysis_samples = end_k - start_k + 1;

if verbose
    fprintf('解析範囲: %.3fs 〜 %.3fs (サンプル %d 〜 %d, 合計 %d サンプル)\n', t_start, t_end, start_k, end_k, analysis_samples);
end

%% 1. 静止期間の分析
fprintf('======================================================\n');
fprintf('1. 静止期間の分析\n');
fprintf('======================================================\n\n');

[bias_stats, sensor_noise] = analyze_static_period(obs, truth, static_samples);

%% 2. バイアス推定精度の分析
fprintf('\n======================================================\n');
fprintf('2. バイアス推定精度の分析\n');
fprintf('======================================================\n\n');

bias_error = analyze_bias_accuracy(obs, truth, static_samples, dt);

%% 3. 初期状態の分析
fprintf('\n======================================================\n');
fprintf('3. 初期状態の分析\n');
fprintf('======================================================\n\n');

    [init_errors, ~] = analyze_initial_state(obs, truth, estimation, static_samples);

%% 4. 誤差の時間発展分析
fprintf('\n======================================================\n');
fprintf('4. 誤差の時間発展分析\n');
fprintf('======================================================\n\n');

error_evolution = analyze_error_evolution(estimation, truth, static_samples, analysis_samples, dt);

%% 5. 発散の診断
fprintf('\n======================================================\n');
fprintf('5. 発散の診断\n');
fprintf('======================================================\n\n');

divergence_info = diagnose_divergence(error_evolution, dt);

%% 6. 総合診断レポート
fprintf('\n======================================================\n');
fprintf('6. 総合診断レポート\n');
fprintf('======================================================\n\n');

generate_diagnosis_report(bias_stats, sensor_noise, bias_error, ...
                         init_errors, error_evolution, divergence_info);

%% 7. 結果のプロット
% プロットは一時的に無効化（デバッグ呼び出しをコメントアウト）
if false % plot_results
    % fprintf('\n結果をプロット中...\n');
    % plot_analysis_results(obs, truth, estimation, static_samples, ...
    %                      analysis_samples, error_evolution, divergence_info);
end
end

%% ===== サブ関数 =====

function [bias_stats, sensor_noise] = analyze_static_period(obs, truth, static_samples)
% 静止期間のセンサーデータを分析

fprintf('--- 加速度計の統計 ---\n');
ax_static = obs.ax(1:static_samples);
ay_static = obs.ay(1:static_samples);
az_static = obs.az(1:static_samples);

bias_stats.accel.mean = [mean(ax_static); mean(ay_static); mean(az_static)];
bias_stats.accel.std = [std(ax_static); std(ay_static); std(az_static)];
bias_stats.accel.norm_mean = mean(sqrt(ax_static.^2 + ay_static.^2 + az_static.^2));

fprintf('  x軸: 平均=%.6f, 標準偏差=%.6f m/s²\n', bias_stats.accel.mean(1), bias_stats.accel.std(1));
fprintf('  y軸: 平均=%.6f, 標準偏差=%.6f m/s²\n', bias_stats.accel.mean(2), bias_stats.accel.std(2));
fprintf('  z軸: 平均=%.6f, 標準偏差=%.6f m/s² (期待値: 9.81)\n', bias_stats.accel.mean(3), bias_stats.accel.std(3));
fprintf('  ノルム: 平均=%.6f m/s²\n\n', bias_stats.accel.norm_mean);

fprintf('--- ジャイロの統計 ---\n');
wx_static = obs.wx(1:static_samples);
wy_static = obs.wy(1:static_samples);
wz_static = obs.wz(1:static_samples);

bias_stats.gyro.mean_deg = [mean(wx_static); mean(wy_static); mean(wz_static)];
bias_stats.gyro.std_deg = [std(wx_static); std(wy_static); std(wz_static)];
bias_stats.gyro.mean_rad = deg2rad(bias_stats.gyro.mean_deg);

fprintf('  x軸: 平均=%.6f, 標準偏差=%.6f deg/s\n', bias_stats.gyro.mean_deg(1), bias_stats.gyro.std_deg(1));
fprintf('  y軸: 平均=%.6f, 標準偏差=%.6f deg/s\n', bias_stats.gyro.mean_deg(2), bias_stats.gyro.std_deg(2));
fprintf('  z軸: 平均=%.6f, 標準偏差=%.6f deg/s (期待値: 0)\n', bias_stats.gyro.mean_deg(3), bias_stats.gyro.std_deg(3));
fprintf('  (rad/s: [%.6f, %.6f, %.6f])\n\n', bias_stats.gyro.mean_rad(1), bias_stats.gyro.mean_rad(2), bias_stats.gyro.mean_rad(3));

% センサーノイズの評価
sensor_noise.accel_std = bias_stats.accel.std;
sensor_noise.gyro_std_deg = bias_stats.gyro.std_deg;
sensor_noise.gyro_std_rad = deg2rad(sensor_noise.gyro_std_deg);

fprintf('--- センサーノイズレベル ---\n');
fprintf('  加速度計: [%.6f, %.6f, %.6f] m/s²\n', sensor_noise.accel_std(1), sensor_noise.accel_std(2), sensor_noise.accel_std(3));
fprintf('  ジャイロ: [%.6f, %.6f, %.6f] deg/s\n', sensor_noise.gyro_std_deg(1), sensor_noise.gyro_std_deg(2), sensor_noise.gyro_std_deg(3));

end

function bias_error = analyze_bias_accuracy(obs, truth, static_samples, dt)
% バイアス補正精度を分析

% バイアス推定（ESKFと同じ方法）
ba_est = [mean(obs.ax(1:static_samples)); 
          mean(obs.ay(1:static_samples)); 
          mean(obs.az(1:static_samples))] - [0; 0; 9.81];

bg_est_deg = [mean(obs.wx(1:static_samples)); 
              mean(obs.wy(1:static_samples)); 
              mean(obs.wz(1:static_samples))];
bg_est_rad = deg2rad(bg_est_deg);

fprintf('--- 推定バイアス ---\n');
fprintf('  ba = [%.6f, %.6f, %.6f] m/s²\n', ba_est(1), ba_est(2), ba_est(3));
fprintf('  bg = [%.6f, %.6f, %.6f] deg/s\n', bg_est_deg(1), bg_est_deg(2), bg_est_deg(3));
fprintf('  bg = [%.6f, %.6f, %.6f] rad/s\n\n', bg_est_rad(1), bg_est_rad(2), bg_est_rad(3));

% 動作期間でのバイアス補正後の統計（静止期間後の全サンプル）
start_idx = static_samples + 1;

% 利用可能なサンプル範囲に合わせて末尾まで解析する
n_obs = length(obs.time);
n_truth = height(truth);
end_idx = min(n_obs, n_truth);

if end_idx < start_idx
    error('解析できるサンプルがありません: start_idx=%d, end_idx=%d', start_idx, end_idx);
end

wx_corr = obs.wx(start_idx:end_idx) - bg_est_deg(1);
wy_corr = obs.wy(start_idx:end_idx) - bg_est_deg(2);
wz_corr = obs.wz(start_idx:end_idx) - bg_est_deg(3);

fprintf('--- バイアス補正後の角速度（動作開始後）---\n');
fprintf('  wx: 平均=%.6f, 標準偏差=%.6f deg/s\n', mean(wx_corr), std(wx_corr));
fprintf('  wy: 平均=%.6f, 標準偏差=%.6f deg/s\n', mean(wy_corr), std(wy_corr));
fprintf('  wz: 平均=%.6f, 標準偏差=%.6f deg/s\n\n', mean(wz_corr), std(wz_corr));

% 真値の角速度と比較（数値微分）
yaw_true = truth.yaw(start_idx:end_idx);
yaw_dot_true = [0; diff(yaw_true)] / dt;

wz_error = wz_corr - yaw_dot_true;

fprintf('--- 真値との比較（yaw方向角速度）---\n');
fprintf('  真値の角速度: 平均=%.6f, 標準偏差=%.6f deg/s\n', mean(yaw_dot_true), std(yaw_dot_true));
fprintf('  角速度誤差: 平均=%.6f, 標準偏差=%.6f deg/s\n', mean(wz_error), std(wz_error));

% 誤差の累積効果
total_time = (length(obs.time) - static_samples) * dt;
accumulated_error = mean(wz_error) * total_time;

fprintf('\n--- 誤差の累積効果 ---\n');
fprintf('  平均角速度誤差: %.6f deg/s\n', mean(wz_error));
fprintf('  全期間の長さ: %.2f 秒\n', total_time);
fprintf('  累積角度誤差: %.2f deg\n', accumulated_error);

bias_error.ba = ba_est;
bias_error.bg_deg = bg_est_deg;
bias_error.bg_rad = bg_est_rad;
bias_error.gyro_mean_error = mean(wz_error);
bias_error.gyro_std_error = std(wz_error);
bias_error.accumulated_angle_error = accumulated_error;

end

function [init_errors, init_state] = analyze_initial_state(obs, truth, estimation, static_samples)
% 初期状態の精度を分析

k = static_samples + 1;  % 動作開始直後

fprintf('--- 初期状態（t=%.4f秒、k=%d）---\n', obs.time(k), k);

% 推定値
init_state.p_est = [estimation.px(k); estimation.py(k); estimation.pz(k)];
init_state.v_est = [estimation.vx(k); estimation.vy(k); estimation.vz(k)];
init_state.euler_est = [estimation.roll(k); estimation.pitch(k); estimation.yaw(k)];

% 真値
init_state.p_true = [truth.x(k); truth.y(k); truth.z(k)];
init_state.v_true = [truth.vx(k); truth.vy(k); truth.vz(k)];
init_state.euler_true = [truth.roll(k); truth.pitch(k); truth.yaw(k)];

% 誤差
init_errors.p = init_state.p_est - init_state.p_true;
init_errors.v = init_state.v_est - init_state.v_true;
init_errors.euler = init_state.euler_est - init_state.euler_true;

fprintf('\n推定値:\n');
fprintf('  位置: [%.6f, %.6f, %.6f] m\n', init_state.p_est(1), init_state.p_est(2), init_state.p_est(3));
fprintf('  速度: [%.6f, %.6f, %.6f] m/s\n', init_state.v_est(1), init_state.v_est(2), init_state.v_est(3));
fprintf('  姿勢: [%.6f, %.6f, %.6f] deg\n', init_state.euler_est(1), init_state.euler_est(2), init_state.euler_est(3));

fprintf('\n真値:\n');
fprintf('  位置: [%.6f, %.6f, %.6f] m\n', init_state.p_true(1), init_state.p_true(2), init_state.p_true(3));
fprintf('  速度: [%.6f, %.6f, %.6f] m/s\n', init_state.v_true(1), init_state.v_true(2), init_state.v_true(3));
fprintf('  姿勢: [%.6f, %.6f, %.6f] deg\n', init_state.euler_true(1), init_state.euler_true(2), init_state.euler_true(3));

fprintf('\n初期誤差:\n');
fprintf('  位置: [%.6f, %.6f, %.6f] m (ノルム: %.6f)\n', ...
    init_errors.p(1), init_errors.p(2), init_errors.p(3), norm(init_errors.p));
fprintf('  速度: [%.6f, %.6f, %.6f] m/s (ノルム: %.6f)\n', ...
    init_errors.v(1), init_errors.v(2), init_errors.v(3), norm(init_errors.v));
fprintf('  姿勢: [%.6f, %.6f, %.6f] deg (ノルム: %.6f)\n', ...
    init_errors.euler(1), init_errors.euler(2), init_errors.euler(3), norm(init_errors.euler));

end

function error_evolution = analyze_error_evolution(estimation, truth, static_samples, analysis_samples, dt)
% 誤差の時間発展を分析

start_k = static_samples + 1;
end_k = min(start_k + analysis_samples - 1, length(estimation.time));

% サンプリング点（対数的に増やす）
sample_points = unique(round(logspace(log10(start_k), log10(end_k), 20)));

n_points = length(sample_points);
error_evolution.time = zeros(n_points, 1);
error_evolution.pos_error = zeros(n_points, 1);
error_evolution.vel_error = zeros(n_points, 1);
error_evolution.att_error = zeros(n_points, 1);
error_evolution.sample_k = sample_points;

fprintf('--- 誤差の時間発展 ---\n');
fprintf('時刻[s]\t\t位置誤差[m]\t速度誤差[m/s]\t姿勢誤差[deg]\n');
fprintf('----------------------------------------------------------\n');

for i = 1:n_points
    k = sample_points(i);
    
    error_evolution.time(i) = estimation.time(k);
    
    pos_err = norm([estimation.px(k) - truth.x(k), ...
                   estimation.py(k) - truth.y(k), ...
                   estimation.pz(k) - truth.z(k)]);
    
    vel_err = norm([estimation.vx(k) - truth.vx(k), ...
                   estimation.vy(k) - truth.vy(k), ...
                   estimation.vz(k) - truth.vz(k)]);
    
    att_err = norm([estimation.roll(k) - truth.roll(k), ...
                   estimation.pitch(k) - truth.pitch(k), ...
                   estimation.yaw(k) - truth.yaw(k)]);
    
    error_evolution.pos_error(i) = pos_err;
    error_evolution.vel_error(i) = vel_err;
    error_evolution.att_error(i) = att_err;
    
    if i <= 10 || mod(i, 5) == 0  % 最初の10点と5点おきに表示
        fprintf('%.4f\t\t%.6f\t%.6f\t%.6f\n', ...
            error_evolution.time(i), pos_err, vel_err, att_err);
    end
end

% 最終時刻を必ず表示
if sample_points(end) ~= end_k
    k = end_k;
    t = estimation.time(k);
    pos_err = norm([estimation.px(k) - truth.x(k), ...
                   estimation.py(k) - truth.y(k), ...
                   estimation.pz(k) - truth.z(k)]);
    vel_err = norm([estimation.vx(k) - truth.vx(k), ...
                   estimation.vy(k) - truth.vy(k), ...
                   estimation.vz(k) - truth.vz(k)]);
    att_err = norm([estimation.roll(k) - truth.roll(k), ...
                   estimation.pitch(k) - truth.pitch(k), ...
                   estimation.yaw(k) - truth.yaw(k)]);
    
    fprintf('%.4f\t\t%.6f\t%.6f\t%.6f (最終)\n', t, pos_err, vel_err, att_err);
end

end

function divergence_info = diagnose_divergence(error_evolution, dt)
% 発散を診断

n_points = length(error_evolution.time);

% 各誤差の増加率を計算
divergence_info.pos_growth_rate = zeros(n_points-1, 1);
divergence_info.vel_growth_rate = zeros(n_points-1, 1);
divergence_info.att_growth_rate = zeros(n_points-1, 1);

for i = 1:n_points-1
    dt_interval = error_evolution.time(i+1) - error_evolution.time(i);
    
    divergence_info.pos_growth_rate(i) = ...
        (error_evolution.pos_error(i+1) - error_evolution.pos_error(i)) / dt_interval;
    divergence_info.vel_growth_rate(i) = ...
        (error_evolution.vel_error(i+1) - error_evolution.vel_error(i)) / dt_interval;
    divergence_info.att_growth_rate(i) = ...
        (error_evolution.att_error(i+1) - error_evolution.att_error(i)) / dt_interval;
end

% 発散閾値
pos_threshold = 0.01;  % m/s
vel_threshold = 0.01;  % (m/s)/s
att_threshold = 0.1;   % deg/s

% 発散開始点の検出
pos_diverge_idx = find(divergence_info.pos_growth_rate > pos_threshold, 1);
vel_diverge_idx = find(divergence_info.vel_growth_rate > vel_threshold, 1);
att_diverge_idx = find(divergence_info.att_growth_rate > att_threshold, 1);

fprintf('\n--- 発散の診断 ---\n');
fprintf('増加率の閾値:\n');
fprintf('  位置: %.4f m/s\n', pos_threshold);
fprintf('  速度: %.4f (m/s)/s\n', vel_threshold);
fprintf('  姿勢: %.4f deg/s\n\n', att_threshold);

if ~isempty(pos_diverge_idx)
    t_diverge = error_evolution.time(pos_diverge_idx);
    fprintf('位置誤差の発散開始: t=%.4f秒\n', t_diverge);
    divergence_info.pos_diverge_time = t_diverge;
else
    fprintf('位置誤差: 発散なし\n');
    divergence_info.pos_diverge_time = inf;
end

if ~isempty(vel_diverge_idx)
    t_diverge = error_evolution.time(vel_diverge_idx);
    fprintf('速度誤差の発散開始: t=%.4f秒\n', t_diverge);
    divergence_info.vel_diverge_time = t_diverge;
else
    fprintf('速度誤差: 発散なし\n');
    divergence_info.vel_diverge_time = inf;
end

if ~isempty(att_diverge_idx)
    t_diverge = error_evolution.time(att_diverge_idx);
    fprintf('姿勢誤差の発散開始: t=%.4f秒\n', t_diverge);
    divergence_info.att_diverge_time = t_diverge;
else
    fprintf('姿勢誤差: 発散なし\n');
    divergence_info.att_diverge_time = inf;
end

% 平均増加率
fprintf('\n平均誤差増加率:\n');
fprintf('  位置: %.6f m/s\n', mean(divergence_info.pos_growth_rate));
fprintf('  速度: %.6f (m/s)/s\n', mean(divergence_info.vel_growth_rate));
fprintf('  姿勢: %.6f deg/s\n', mean(divergence_info.att_growth_rate));

end

function generate_diagnosis_report(bias_stats, sensor_noise, bias_error, ...
                                   init_errors, error_evolution, divergence_info)
% 総合診断レポートを生成

fprintf('=== 主な問題点 ===\n\n');

issues = {};
issue_count = 0;

% 1. センサーノイズのチェック
if any(sensor_noise.accel_std > 0.1)
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 加速度計ノイズが大きい (%.4f m/s²)', ...
        issue_count, max(sensor_noise.accel_std));
end

if any(sensor_noise.gyro_std_deg > 1.0)
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: ジャイロノイズが大きい (%.4f deg/s)', ...
        issue_count, max(sensor_noise.gyro_std_deg));
end

% 2. バイアス誤差のチェック
if abs(bias_error.gyro_mean_error) > 0.1
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: ジャイロバイアス補正後も大きな誤差 (%.4f deg/s)', ...
        issue_count, bias_error.gyro_mean_error);
    issues{end+1} = sprintf('  → %.2f秒間で約%.2f度の累積誤差が発生', ...
        error_evolution.time(end), bias_error.accumulated_angle_error);
end

% 3. 初期状態誤差のチェック
if norm(init_errors.p) > 0.01
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 初期位置誤差が大きい (%.6f m)', ...
        issue_count, norm(init_errors.p));
end

if norm(init_errors.v) > 0.01
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 初期速度誤差が大きい (%.6f m/s)', ...
        issue_count, norm(init_errors.v));
end

if norm(init_errors.euler) > 1.0
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 初期姿勢誤差が大きい (%.4f deg)', ...
        issue_count, norm(init_errors.euler));
end

% 4. 発散のチェック
if divergence_info.pos_diverge_time < inf
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 位置が発散 (t=%.4f秒から)', ...
        issue_count, divergence_info.pos_diverge_time);
end

if divergence_info.vel_diverge_time < inf
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 速度が発散 (t=%.4f秒から)', ...
        issue_count, divergence_info.vel_diverge_time);
end

if divergence_info.att_diverge_time < inf
    issue_count = issue_count + 1;
    issues{end+1} = sprintf('問題%d: 姿勢が発散 (t=%.4f秒から)', ...
        issue_count, divergence_info.att_diverge_time);
end

% 問題点を表示
if isempty(issues)
    fprintf('特に大きな問題は検出されませんでした。\n\n');
else
    for i = 1:length(issues)
        fprintf('%s\n', issues{i});
    end
    fprintf('\n');
end

% 推奨事項
fprintf('=== 推奨事項 ===\n\n');

if any(sensor_noise.accel_std > 0.1) || any(sensor_noise.gyro_std_deg > 1.0)
    fprintf('・センサーノイズが大きい → ノイズフィルタの適用を検討\n');
    fprintf('  - ローパスフィルタの追加\n');
    fprintf('  - プロセスノイズ行列Qの調整\n');
end

if abs(bias_error.gyro_mean_error) > 0.1
    fprintf('・ジャイロバイアスの補正精度が低い\n');
    fprintf('  - 静止期間を延長してより正確なバイアス推定\n');
    fprintf('  - バイアスをオンラインで推定・更新\n');
    fprintf('  - ランダムウォークモデルの導入\n');
end

if norm(init_errors.v) > 0.01 || norm(init_errors.euler) > 1.0
    fprintf('・初期状態の精度が低い\n');
    fprintf('  - 初期共分散行列Pの調整\n');
    fprintf('  - より正確な初期姿勢推定手法の採用\n');
end

if divergence_info.att_diverge_time < divergence_info.pos_diverge_time && ...
   divergence_info.att_diverge_time < divergence_info.vel_diverge_time
    fprintf('・姿勢が最初に発散 → 姿勢推定の改善が最優先\n');
    fprintf('  - 観測更新の頻度を増やす\n');
    fprintf('  - 観測ノイズ行列Rの調整\n');
elseif divergence_info.vel_diverge_time < divergence_info.pos_diverge_time
    fprintf('・速度が位置より先に発散 → 速度推定の改善が重要\n');
    fprintf('  - GPS速度の観測を追加\n');
    fprintf('  - 速度に関するプロセスノイズの調整\n');
end

fprintf('\n');

end

function plot_analysis_results(obs, truth, estimation, static_samples, ...
                               analysis_samples, error_evolution, divergence_info)
% 分析結果をタブ付きの1つの figure にプロット（uitab を使用）

figure
% Subplot (1,1): 誤差の時間発展 (log scale)
ax1 = subplot(2,2,1);
semilogy(ax1, error_evolution.time, error_evolution.pos_error, 'b-', 'LineWidth', 1.5);
hold(ax1,'on');
semilogy(ax1, error_evolution.time, error_evolution.vel_error, 'r-', 'LineWidth', 1.5);
semilogy(ax1, error_evolution.time, error_evolution.att_error, 'g-', 'LineWidth', 1.5);
if divergence_info.pos_diverge_time < inf
    xline(divergence_info.pos_diverge_time, 'b--', '位置発散');
end
if divergence_info.vel_diverge_time < inf
    xline(divergence_info.vel_diverge_time, 'r--', '速度発散');
end
if divergence_info.att_diverge_time < inf
    xline(divergence_info.att_diverge_time, 'g--', '姿勢発散');
end
hold(ax1,'off');
grid(ax1,'on');
xlabel(ax1,'時刻 [s]');
ylabel(ax1,'誤差 (対数スケール)');
title(ax1,'誤差の時間発展');
legend(ax1,{'位置誤差 [m]','速度誤差 [m/s]','姿勢誤差 [deg]'}, 'Location','best');

% Subplot (1,2): 位置の軌跡
ax2 = subplot(2,2,2);
plot(ax2, truth.x, truth.y, 'b-', 'LineWidth', 1.5);
hold(ax2,'on');
plot(ax2, estimation.px, estimation.py, 'r--', 'LineWidth', 1.5);
hold(ax2,'off');
grid(ax2,'on');
axis(ax2,'equal');
xlabel(ax2,'X [m]');
ylabel(ax2,'Y [m]');
title(ax2,'位置の軌跡（XY平面）');
legend(ax2,{'真値','推定'}, 'Location','best');

% Subplot (2,1): 姿勢(Yaw) の比較
ax3 = subplot(2,2,3);
start_k = static_samples + 1;
end_k = min(start_k + analysis_samples - 1, length(estimation.time));
time_range = estimation.time(start_k:end_k);
plot(ax3, time_range, truth.yaw(start_k:end_k), 'b-', 'LineWidth', 1.5);
hold(ax3,'on');
plot(ax3, time_range, estimation.yaw(start_k:end_k), 'r--', 'LineWidth', 1.5);
hold(ax3,'off');
grid(ax3,'on');
xlabel(ax3,'時刻 [s]');
ylabel(ax3,'Yaw [deg]');
title(ax3,'姿勢（Yaw角）の比較');
legend(ax3,{'真値','推定'}, 'Location','best');

% Subplot (2,2): 速度ノルムの比較
ax4 = subplot(2,2,4);
vel_true_norm = sqrt(truth.vx(start_k:end_k).^2 + truth.vy(start_k:end_k).^2);
vel_est_norm = sqrt(estimation.vx(start_k:end_k).^2 + estimation.vy(start_k:end_k).^2);
plot(ax4, time_range, vel_true_norm, 'b-', 'LineWidth', 1.5);
hold(ax4,'on');
plot(ax4, time_range, vel_est_norm, 'r--', 'LineWidth', 1.5);
hold(ax4,'off');
grid(ax4,'on');
xlabel(ax4,'時刻 [s]');
ylabel(ax4,'速度ノルム [m/s]');
title(ax4,'速度ノルムの比較');
legend(ax4,{'真値','推定'}, 'Location','best');

% タイトル（figure上部）
% sgtitle が利用可能なら設定する（なければ無視）
try
    sgtitle('推定失敗の分析結果', 'FontSize', 14, 'FontWeight', 'bold');
catch
    % sgtitle が無ければ何もしない
end

end
