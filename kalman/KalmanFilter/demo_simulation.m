function demo_simulation()
% DEMO_SIMULATION - デモンストレーション用の短いシミュレーション
% 100ポイントで動作確認

fprintf('=== デモシミュレーション開始 ===\n');

% パス設定
current_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(current_dir, '..', 'GenerateData'));

% CSVデータ読み込み（最初の100ポイントのみ）
csv_file = fullfile(current_dir, '..', '..', 'sim_data.csv');
[sensor_data_full, truth_data_full] = read_csv_data(csv_file);

% データを100ポイントに制限
DEMO_POINTS = 100;
sensor_data = limit_data(sensor_data_full, DEMO_POINTS);
truth_data = limit_data(truth_data_full, DEMO_POINTS);

fprintf('デモ用データ: %d ポイント\n', length(sensor_data.t));

% パラメータ設定
params = config_params();

% UKFインスタンス作成
ukf_vel = UKF_Calculator(6, 3);
ukf_att = UKF_Calculator(6, 6);
ukf_pos = UKF_Calculator(6, 6);

% 結果保存
results = struct();
results.vel = [];
results.att = [];
results.pos = [];

% 状態
prev_vel_state = [];
prev_att_state = [];
prev_pos_state = [];

fprintf('処理開始...\n');

% 簡略化ループ
for i = 1:DEMO_POINTS
    % 速度推定（毎回）
    vel_result = velocity_estimator(sensor_data, params, ukf_vel, i, prev_vel_state);
    prev_vel_state = vel_result;
    results.vel = [results.vel; vel_result.velocity'];
    
    % 姿勢推定（4回に1回）
    if mod(i-1, 4) == 0
        att_result = attitude_estimator(sensor_data, params, ukf_att, i, prev_att_state, vel_result);
        prev_att_state = att_result;
        results.att = [results.att; att_result.attitude'];
        fprintf('姿勢推定 %d: Yaw=%.2f rad\n', ceil(i/4), att_result.attitude(3));
    end
    
    % 位置推定（40回に1回）
    if mod(i-1, 40) == 0
        pos_result = position_estimator(sensor_data, params, ukf_pos, i, prev_pos_state, {vel_result}, {prev_att_state});
        prev_pos_state = pos_result;
        results.pos = [results.pos; pos_result.position'];
        fprintf('位置推定 %d: [%.2f, %.2f, %.2f] m\n', ceil(i/40), pos_result.position);
    end
    
    if mod(i, 20) == 0
        fprintf('進行状況: %d/%d\n', i, DEMO_POINTS);
    end
end

fprintf('デモシミュレーション完了!\n');

% 簡単な結果表示
fprintf('\n=== 結果サマリー ===\n');
fprintf('速度推定: %d回\n', size(results.vel, 1));
fprintf('姿勢推定: %d回\n', size(results.att, 1));
fprintf('位置推定: %d回\n', size(results.pos, 1));

% 最終値表示
if ~isempty(results.vel)
    fprintf('最終速度推定: [%.3f, %.3f, %.3f] m/s\n', results.vel(end, :));
end
if ~isempty(results.att)
    fprintf('最終姿勢推定: [%.3f, %.3f, %.3f] rad\n', results.att(end, :));
end
if ~isempty(results.pos)
    fprintf('最終位置推定: [%.3f, %.3f, %.3f] m\n', results.pos(end, :));
end

% 真値との比較
if size(results.vel, 1) > 0
    vel_error = norm(results.vel(end, :) - truth_data.vel(DEMO_POINTS, 1:3));
    fprintf('最終速度誤差: %.4f m/s\n', vel_error);
end

if size(results.pos, 1) > 0
    pos_error = norm(results.pos(end, :) - truth_data.pos(DEMO_POINTS, :));
    fprintf('最終位置誤差: %.4f m\n', pos_error);
end

fprintf('===================\n');

end

function limited_data = limit_data(data, max_points)
% データを指定ポイント数に制限
limited_data = struct();
fields = fieldnames(data);
for i = 1:length(fields)
    field = fields{i};
    original = data.(field);
    if size(original, 1) >= max_points
        limited_data.(field) = original(1:max_points, :);
    else
        limited_data.(field) = original;
    end
end
end