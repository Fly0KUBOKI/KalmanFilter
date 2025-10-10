function run_simulation()
% RUN_SIMULATION - カスケードUKFシミュレーション実行
% 速度→姿勢→位置の順でカスケード推定を実行


% パラメータ定義
VEL_FACTOR = 1;    % 速度推定頻度（毎回）
ATT_FACTOR = 4;    % 姿勢推定頻度（4回に1回）
POS_FACTOR = 40;   % 位置推定頻度（40回に1回）

% --- 表示切り替え用define ---
ENABLE_VELOCITY = true;   % 速度グラフを表示
ENABLE_ATTITUDE = false;   % 姿勢グラフを表示
ENABLE_POSITION = false;   % 位置グラフを表示

fprintf('=== カスケードUKFシミュレーション開始 ===\n');

% パス設定
current_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(current_dir, '..', 'GenerateData'));

% CSVデータ読み込み
csv_file = fullfile(current_dir, '..', '..', 'sim_data.csv');
[sensor_data, truth_data] = read_csv_data(csv_file);

% パラメータ設定
params = config_params();

% If data comes from CSV, estimate sensor noise std from data and override params where it
% appears mismatched. This prevents underestimating R which causes huge innovations.
try
    if isfield(params, 'data') && isfield(params.data, 'source') && strcmp(params.data.source, 'csv')
        % empirical std over a window (use entire file for now)
        accel_std = std(sensor_data.accel3, 0, 1);
        gyro_std = std(sensor_data.gyro3, 0, 1);
        mag_std = std(sensor_data.mag3, 0, 1);
        gps_std = std(sensor_data.gps, 0, 1);
        baro_std = std(sensor_data.baro, 0, 1);
        fprintf('Calibrating params.noise from CSV: accel_std=[%.3g %.3g %.3g], gyro_std=[%.3g %.3g %.3g], gps_std=[%.3g %.3g], baro_std=%.3g\n', ...
            accel_std(1), accel_std(2), accel_std(3), gyro_std(1), gyro_std(2), gyro_std(3), gps_std(1), gps_std(2), baro_std);
        params.noise.accel3 = double(accel_std);
        params.noise.gyro3 = double(gyro_std);
        params.noise.mag3 = double(mag_std);
        params.noise.gps = mean(double(gps_std));
        params.noise.baro = double(baro_std);
    end
catch ME
    fprintf('Noise calibration failed: %s\n', ME.message);
end

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
% 制御用のアプリデータを初期化（Enterでstop/start、'q'で終了、図を閉じると終了）
control.running = true;
control.quit = false;
setappdata(0, 'sim_control', control);

% 既存のデフォルトコールバックを保存しておく
oldKeyPress = get(0, 'DefaultFigureKeyPressFcn');
oldCloseReq = get(0, 'DefaultFigureCloseRequestFcn');
% 新しいデフォルトコールバックを設定（これにより plot_results/plot_final_results 等で作成される図にも適用される）
set(0, 'DefaultFigureKeyPressFcn', @sim_keypress_cb);
set(0, 'DefaultFigureCloseRequestFcn', @sim_close_cb);

fprintf('データ処理中... (Enter: pause/resume, q: quit, 図を閉じる: 終了)\n');

for i = 1:length(sensor_data.t)
    if mod(i, 1000) == 0
        fprintf('処理中: %d/%d (%.1f%%)\n', i, length(sensor_data.t), i/length(sensor_data.t)*100);
    end

    % ユーザ制御を確認
    ctrl = getappdata(0, 'sim_control');
    if isempty(ctrl)
        ctrl.running = true; ctrl.quit = false; setappdata(0,'sim_control',ctrl);
    end
    if isfield(ctrl, 'quit') && ctrl.quit
        fprintf('ユーザ要求により中断します (quit)\n');
        break;
    end
    % 一時停止ループ: pause を使わず drawnow でコールバック処理を行う
    while isfield(ctrl, 'running') && ~ctrl.running
        drawnow; % キーボードコールバック等を処理する
        ctrl = getappdata(0, 'sim_control');
        if isempty(ctrl)
            break;
        end
        if isfield(ctrl, 'quit') && ctrl.quit
            break;
        end
    end
    if isfield(ctrl, 'quit') && ctrl.quit
        break;
    end
    
    % 速度推定（毎回実行）
    if mod(i-1, VEL_FACTOR) == 0
        % pass latest attitude estimate if available (use most recent att_results)
        if ~isempty(att_results)
            latest_att = [];
            % find most recent non-empty attitude
            for j = length(att_results):-1:1
                if ~isempty(att_results{j})
                    latest_att = att_results{j};
                    break;
                end
            end
        else
            latest_att = [];
        end
        vel_results{i} = velocity_estimator(sensor_data, params, ukf_vel, i, prev_vel_state, latest_att);
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
        
        % プロット更新（表示オプションを渡す）

        % plot_resultsの表示内容をdefineで切り替え
        plot_opts = struct('enable_velocity', ENABLE_VELOCITY, 'enable_attitude', ENABLE_ATTITUDE, 'enable_position', ENABLE_POSITION);
        plot_results(vel_results, att_results, pos_results, truth_data, pos_idx, plot_opts);
        
        pos_idx = pos_idx + 1;
    end
end

fprintf('シミュレーション完了!\n');

% 結果を保存
results_file = fullfile(current_dir, 'simulation_results.mat');
save(results_file, 'vel_results', 'att_results', 'pos_results', 'truth_data', 'sensor_data');
fprintf('結果を保存しました: %s\n', results_file);

% 最終プロット

% 最終プロットもdefineに従う
try
    % 最終結果は常に全て表示
    plot_final_results(vel_results, att_results, pos_results, truth_data, sensor_data, struct('enable_velocity',true,'enable_attitude',true,'enable_position',true));
catch ME
    fprintf('プロット作成でエラーが発生しました: %s\n', ME.message);
    fprintf('結果はファイルに保存されています: %s\n', results_file);
end

% 後片付け: デフォルトコールバックを復元し、アプリデータを消す
try
    set(0, 'DefaultFigureKeyPressFcn', oldKeyPress);
    set(0, 'DefaultFigureCloseRequestFcn', oldCloseReq);
catch
    % 無視
end
if isappdata(0, 'sim_control')
    rmappdata(0, 'sim_control');
end

end

% --- コールバック関数 ---
function sim_keypress_cb(~, event)
    % Enter で一時停止/再開、'q' で終了
    try
        ctrl = getappdata(0, 'sim_control');
        if isempty(ctrl)
            ctrl.running = true; ctrl.quit = false;
        end
        key = event.Key;
        if strcmpi(key, 'return') || strcmpi(key, 'enter')
            ctrl.running = ~ctrl.running;
            if ctrl.running
                fprintf('再開: Enterが押されました\n');
            else
                fprintf('一時停止: Enterが押されました\n');
            end
        elseif strcmpi(key, 'q')
            ctrl.quit = true;
            fprintf('終了要求: q が押されました\n');
        end
        setappdata(0, 'sim_control', ctrl);
    catch
        % 無視
    end
end

function sim_close_cb(src, ~)
    % 図が閉じられたら終了フラグを立ててから削除
    try
        ctrl = getappdata(0, 'sim_control');
        if isempty(ctrl)
            ctrl.running = false; ctrl.quit = true;
        else
            ctrl.quit = true;
        end
        setappdata(0, 'sim_control', ctrl);
    catch
        % 無視
    end
    % デフォルトの閉じ処理を実行
    delete(src);
end

function vel_avg = calculate_velocity_average(vel_buffer)
% 速度推定結果の平均を計算
vel_sum = zeros(3, 1, 'single');
accel_sum = zeros(3, 1, 'single');
velP_sum = zeros(3,3, 'single');
count = 0;
for i = 1:length(vel_buffer)
    if ~isempty(vel_buffer{i})
        vel_sum = vel_sum + vel_buffer{i}.velocity;
        % vel_buffer entries include acceleration estimate
        if isfield(vel_buffer{i}, 'acceleration') && ~isempty(vel_buffer{i}.acceleration)
            accel_sum = accel_sum + vel_buffer{i}.acceleration;
        end
        if isfield(vel_buffer{i}, 'P') && ~isempty(vel_buffer{i}.P)
            % P is 6x6; take velocity part (1:3,1:3)
            Pvel = single(vel_buffer{i}.P(1:3,1:3));
            velP_sum = velP_sum + Pvel;
        end
        count = count + 1;
    end
end
if count > 0
    vel_avg = struct();
    vel_avg.velocity = vel_sum / count;
    % 平均加速度を返す（以前はゼロ固定だった）
    vel_avg.acceleration = accel_sum / count;
    % 平均共分散（速度部分）を返す
    vel_avg.P = velP_sum / count;
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