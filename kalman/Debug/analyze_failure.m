function analyze_failure(sensor_file, truth_file, estimation_file, varargin)
% analyze_failure - 推定失敗の原因分析
%
% 使用法:
%   analyze_failure(sensor_file, truth_file, estimation_file)
%   analyze_failure(..., 'StaticTime', 2.0, 'dt', 0.0025, 'PlotResults', true)
%
% 入力:
%   sensor_file      - センサーデータCSVファイル
%   truth_file       - 真値データCSVファイル
%   estimation_file  - 推定結果CSVファイル
%
% オプション:
%   'StaticTime'     - 静止期間[秒] (デフォルト: 2.0)
%   'dt'             - サンプリング周期[秒] (デフォルト: 0.0025)
%   'PlotResults'    - プロット表示 (デフォルト: true)

p = inputParser;
addRequired(p, 'sensor_file');
addRequired(p, 'truth_file');
addRequired(p, 'estimation_file');
addParameter(p, 'StaticTime', 2.0);
addParameter(p, 'dt', 0.0025);
addParameter(p, 'PlotResults', true);
parse(p, sensor_file, truth_file, estimation_file, varargin{:});

static_time = p.Results.StaticTime;
dt = p.Results.dt;
plot_results = p.Results.PlotResults;

addpath('GenerateData');

% データ読み込み
obs = read_csv(sensor_file);
truth = readtable(truth_file);
estimation = readtable(estimation_file);

static_samples = floor(static_time / dt);

%% 1. 静止期間の分析
fprintf('======================================================\n');
fprintf('1. 静止期間の分析\n');
fprintf('======================================================\n\n');

[bias_stats, sensor_noise] = static_analysis(obs, truth, static_samples);

%% 2. バイアス推定精度の分析
fprintf('\n======================================================\n');
fprintf('2. バイアス推定精度の分析\n');
fprintf('======================================================\n\n');

bias_error = bias_analysis(obs, truth, static_samples, dt);

%% 3. 初期状態の分析
fprintf('\n======================================================\n');
fprintf('3. 初期状態の分析\n');
fprintf('======================================================\n\n');

init_errors = initial_state_analysis(obs, truth, estimation, static_samples);

%% 4. 誤差の時間発展分析
fprintf('\n======================================================\n');
fprintf('4. 誤差の時間発展分析\n');
fprintf('======================================================\n\n');

error_evolution = error_evolution_analysis(estimation, truth, static_samples, dt);

%% 5. 発散の診断
fprintf('\n======================================================\n');
fprintf('5. 発散の診断\n');
fprintf('======================================================\n\n');

divergence_info = divergence_diagnosis(error_evolution, dt);

%% 6. 総合診断レポート
fprintf('\n======================================================\n');
fprintf('6. 総合診断レポート\n');
fprintf('======================================================\n\n');

diagnosis_report(bias_stats, sensor_noise, bias_error, init_errors, error_evolution, divergence_info);

%% 7. 結果のプロット
% プロットは一時的に無効化（デバッグ呼び出しをコメントアウト）
if false % plot_results
    % fprintf('\n結果をプロット中...\n');
    % plot_error_evolution(error_evolution, divergence_info);
    % plot_trajectory(truth, estimation);
    % plot_yaw(truth, estimation, static_samples);
    % plot_velocity(truth, estimation, static_samples);
end
end
