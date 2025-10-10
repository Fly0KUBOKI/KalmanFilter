function run_simulation()
% RUN_SIMULATION - カスケードUKFシミュレーション実行
% 速度→姿勢→位置の順でカスケード推定を実行

% パラメータ定義
VEL_FACTOR = 1;    % 速度推定頻度（毎回）
ATT_FACTOR = 4;    % 姿勢推定頻度（4回に1回）
POS_FACTOR = 40;   % 位置推定頻度（40回に1回）

fprintf('=== カスケードUKFシミュレーション開始 ===\n');

% パス設定
current_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(current_dir, '..', 'GenerateData'));

% CSVデータ読み込み
csv_file = fullfile(current_dir, '..', '..', 'sim_data.csv');
[sensor_data, truth_data] = read_csv_data(csv_file);

% パラメータ設定
params = config_params();

% UKFインスタンス作成
ukf_vel = UKF_Calculator(6, 3);  % 速度推定: 6状態, 3観測
ukf_att = UKF_Calculator(6, 6);  % 姿勢推定: 6状態, 6観測  
ukf_pos = UKF_Calculator(6, 6);  % 位置推定: 6状態, 6観測

% 結果保存用
vel_results = cell(length(sensor_data.t), 1);
att_results = cell(ceil(length(sensor_data.t) / ATT_FACTOR), 1);
pos_results = cell(ceil(length(sensor_data.t) / POS_FACTOR), 1);

% 前回状態
prev_vel_state = [];
prev_att_state = [];
prev_pos_state = [];

% 推定値バッファ
vel_buffer = cell(ATT_FACTOR, 1);
att_buffer = cell(POS_FACTOR / ATT_FACTOR, 1);

% メインループ
att_idx = 1;
pos_idx = 1;
vel_buf_idx = 1;
att_buf_idx = 1;

fprintf('データ処理中...\n');
for i = 1:length(sensor_data.t)
    if mod(i, 1000) == 0
        fprintf('処理中: %d/%d (%.1f%%)\n', i, length(sensor_data.t), i/length(sensor_data.t)*100);
    end
    
    % 速度推定（毎回実行）
    if mod(i-1, VEL_FACTOR) == 0
        vel_results{i} = velocity_estimator(sensor_data, params, ukf_vel, i, prev_vel_state);
        prev_vel_state = vel_results{i};
        
        % 速度推定結果をバッファに保存
        vel_buffer{vel_buf_idx} = vel_results{i};
        vel_buf_idx = vel_buf_idx + 1;
        if vel_buf_idx > ATT_FACTOR
            vel_buf_idx = 1;
        end
    end
    
    % 姿勢推定（ATT_FACTOR回に1回実行）
    if mod(i-1, ATT_FACTOR) == 0
        % 速度推定結果の平均を計算
        vel_avg = calculate_velocity_average(vel_buffer);
        
        att_results{att_idx} = attitude_estimator(sensor_data, params, ukf_att, i, prev_att_state, vel_avg);
        prev_att_state = att_results{att_idx};
        
        % 姿勢推定結果をバッファに保存
        att_buffer{att_buf_idx} = att_results{att_idx};
        att_buf_idx = att_buf_idx + 1;
        if att_buf_idx > (POS_FACTOR / ATT_FACTOR)
            att_buf_idx = 1;
        end
        
        att_idx = att_idx + 1;
    end
    
    % 位置推定（POS_FACTOR回に1回実行）
    if mod(i-1, POS_FACTOR) == 0
        % 速度と姿勢推定結果の平均を計算
        vel_avg = calculate_velocity_average(vel_buffer);
        att_avg = calculate_attitude_average(att_buffer);
        
        pos_results{pos_idx} = position_estimator(sensor_data, params, ukf_pos, i, prev_pos_state, {vel_avg}, {att_avg});
        prev_pos_state = pos_results{pos_idx};
        
        % プロット更新
        plot_results(vel_results, att_results, pos_results, truth_data, pos_idx);
        
        pos_idx = pos_idx + 1;
    end
end

fprintf('シミュレーション完了!\n');

% 結果を保存
results_file = fullfile(current_dir, 'simulation_results.mat');
save(results_file, 'vel_results', 'att_results', 'pos_results', 'truth_data', 'sensor_data');
fprintf('結果を保存しました: %s\n', results_file);

% 最終プロット
try
    plot_final_results(vel_results, att_results, pos_results, truth_data, sensor_data);
catch ME
    fprintf('プロット作成でエラーが発生しました: %s\n', ME.message);
    fprintf('結果はファイルに保存されています: %s\n', results_file);
end

end

function vel_avg = calculate_velocity_average(vel_buffer)
% 速度推定結果の平均を計算
vel_sum = zeros(3, 1, 'single');
count = 0;
for i = 1:length(vel_buffer)
    if ~isempty(vel_buffer{i})
        vel_sum = vel_sum + vel_buffer{i}.velocity;
        count = count + 1;
    end
end
if count > 0
    vel_avg = struct();
    vel_avg.velocity = vel_sum / count;
    vel_avg.acceleration = zeros(3, 1, 'single');
else
    vel_avg = [];
end
end

function att_avg = calculate_attitude_average(att_buffer)
% 姿勢推定結果の平均を計算
att_sum = zeros(3, 1, 'single');
count = 0;
for i = 1:length(att_buffer)
    if ~isempty(att_buffer{i})
        att_sum = att_sum + att_buffer{i}.attitude;
        count = count + 1;
    end
end
if count > 0
    att_avg = struct();
    att_avg.attitude = att_sum / count;
else
    att_avg = [];
end
end