function run_optimized_simulation()
% RUN_OPTIMIZED_SIMULATION - メモリ効率を改善したカスケードUKFシミュレーション
% 大量データ処理に対応

% パラメータ定義
VEL_FACTOR = 1;    % 速度推定頻度（毎回）
ATT_FACTOR = 4;    % 姿勢推定頻度（4回に1回）
POS_FACTOR = 40;   % 位置推定頻度（40回に1回）

% バッチ処理サイズ（メモリ節約）
BATCH_SIZE = 5000;

fprintf('=== 最適化カスケードUKFシミュレーション開始 ===\n');

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

% 結果保存用（重要なデータのみ保存）
total_points = length(sensor_data.t);
vel_summary = struct('timestamps', [], 'velocities', [], 'errors', []);
att_summary = struct('timestamps', [], 'attitudes', [], 'errors', []);
pos_summary = struct('timestamps', [], 'positions', [], 'errors', []);

% 前回状態
prev_vel_state = [];
prev_att_state = [];
prev_pos_state = [];

% 推定値バッファ
vel_buffer = cell(ATT_FACTOR, 1);
att_buffer = cell(POS_FACTOR / ATT_FACTOR, 1);

% カウンタ
att_idx = 1;
pos_idx = 1;
vel_buf_idx = 1;
att_buf_idx = 1;

% バッチ処理
num_batches = ceil(total_points / BATCH_SIZE);
fprintf('総データ点数: %d, バッチ数: %d\n', total_points, num_batches);

for batch = 1:num_batches
    start_idx = (batch - 1) * BATCH_SIZE + 1;
    end_idx = min(batch * BATCH_SIZE, total_points);
    
    fprintf('バッチ %d/%d 処理中 (データ点 %d-%d)...\n', batch, num_batches, start_idx, end_idx);
    
    for i = start_idx:end_idx
        % 進行状況表示（各バッチ内で）
        if mod(i - start_idx + 1, 500) == 0
            fprintf('  バッチ内進行: %d/%d\n', i - start_idx + 1, end_idx - start_idx + 1);
        end
        
        % 速度推定（毎回実行）
        if mod(i-1, VEL_FACTOR) == 0
            vel_result = velocity_estimator(sensor_data, params, ukf_vel, i, prev_vel_state);
            prev_vel_state = vel_result;
            
            % サマリーに保存（間引いて保存）
            if mod(i, 10) == 0 || i <= 100
                vel_summary.timestamps = [vel_summary.timestamps; sensor_data.t(i)];
                vel_summary.velocities = [vel_summary.velocities; vel_result.velocity'];
                
                % 真値との誤差計算
                if i <= length(truth_data.vel)
                    error_norm = norm(vel_result.velocity - truth_data.vel(i, 1:3)');
                    vel_summary.errors = [vel_summary.errors; error_norm];
                else
                    vel_summary.errors = [vel_summary.errors; NaN];
                end
            end
            
            % 速度推定結果をバッファに保存
            vel_buffer{vel_buf_idx} = vel_result;
            vel_buf_idx = vel_buf_idx + 1;
            if vel_buf_idx > ATT_FACTOR
                vel_buf_idx = 1;
            end
        end
        
        % 姿勢推定（ATT_FACTOR回に1回実行）
        if mod(i-1, ATT_FACTOR) == 0
            % 速度推定結果の平均を計算
            vel_avg = calculate_velocity_average(vel_buffer);
            
            att_result = attitude_estimator(sensor_data, params, ukf_att, i, prev_att_state, vel_avg);
            prev_att_state = att_result;
            
            % サマリーに保存
            att_summary.timestamps = [att_summary.timestamps; att_result.timestamp];
            att_summary.attitudes = [att_summary.attitudes; att_result.attitude'];
            
            % 真値との誤差計算（Yaw角のみ）
            if i <= length(truth_data.theta)
                yaw_error = abs(wrap_to_pi(att_result.attitude(3) - truth_data.theta(i)));
                att_summary.errors = [att_summary.errors; yaw_error];
            else
                att_summary.errors = [att_summary.errors; NaN];
            end
            
            % 姿勢推定結果をバッファに保存
            att_buffer{att_buf_idx} = att_result;
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
            
            pos_result = position_estimator(sensor_data, params, ukf_pos, i, prev_pos_state, {vel_avg}, {att_avg});
            prev_pos_state = pos_result;
            
            % サマリーに保存
            pos_summary.timestamps = [pos_summary.timestamps; pos_result.timestamp];
            pos_summary.positions = [pos_summary.positions; pos_result.position'];
            
            % 真値との誤差計算
            if i <= size(truth_data.pos, 1)
                pos_error = norm(pos_result.position - truth_data.pos(i, :)');
                pos_summary.errors = [pos_summary.errors; pos_error];
            else
                pos_summary.errors = [pos_summary.errors; NaN];
            end
            
            fprintf('    位置推定 %d: [%.2f, %.2f, %.2f] m (誤差: %.2f m)\n', ...
                pos_idx, pos_result.position, pos_summary.errors(end));
            
            pos_idx = pos_idx + 1;
        end
    end
    
    % メモリクリーンアップ（必要に応じて）
    if mod(batch, 5) == 0
        fprintf('  メモリクリーンアップ実行\n');
        clear vel_result att_result pos_result;
    end
end

fprintf('最適化シミュレーション完了!\n');

% 結果統計
fprintf('\n=== 結果統計 ===\n');
fprintf('速度推定データ点数: %d\n', length(vel_summary.timestamps));
fprintf('姿勢推定データ点数: %d\n', length(att_summary.timestamps));
fprintf('位置推定データ点数: %d\n', length(pos_summary.timestamps));

if ~isempty(vel_summary.errors)
    valid_vel_errors = vel_summary.errors(~isnan(vel_summary.errors));
    if ~isempty(valid_vel_errors)
        fprintf('速度推定 - 平均誤差: %.4f m/s, 最大誤差: %.4f m/s\n', ...
            mean(valid_vel_errors), max(valid_vel_errors));
    end
end

if ~isempty(att_summary.errors)
    valid_att_errors = att_summary.errors(~isnan(att_summary.errors));
    if ~isempty(valid_att_errors)
        fprintf('姿勢推定 - 平均Yaw誤差: %.4f rad (%.1f deg), 最大誤差: %.4f rad (%.1f deg)\n', ...
            mean(valid_att_errors), rad2deg(mean(valid_att_errors)), ...
            max(valid_att_errors), rad2deg(max(valid_att_errors)));
    end
end

if ~isempty(pos_summary.errors)
    valid_pos_errors = pos_summary.errors(~isnan(pos_summary.errors));
    if ~isempty(valid_pos_errors)
        fprintf('位置推定 - 平均誤差: %.4f m, 最大誤差: %.4f m\n', ...
            mean(valid_pos_errors), max(valid_pos_errors));
    end
end

% 結果を保存
results_file = fullfile(current_dir, 'optimized_simulation_results.mat');
save(results_file, 'vel_summary', 'att_summary', 'pos_summary', 'truth_data', 'sensor_data');
fprintf('結果を保存しました: %s\n', results_file);

% 最適化プロット
try
    create_optimized_plot(vel_summary, att_summary, pos_summary, truth_data);
catch ME
    fprintf('プロット作成でエラーが発生しました: %s\n', ME.message);
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

function create_optimized_plot(vel_summary, att_summary, pos_summary, truth_data)
% 最適化されたプロット作成

figure('Name', '最適化シミュレーション結果', 'Position', [50, 50, 1400, 900]);

% 位置軌跡
subplot(2, 3, 1);
hold on;
title('位置軌跡 (XY平面)');
xlabel('X [m]');
ylabel('Y [m]');
grid on;
axis equal;

% 真値軌跡（間引き表示）
step = max(1, floor(length(truth_data.pos) / 1000));
plot(truth_data.pos(1:step:end, 1), truth_data.pos(1:step:end, 2), 'k-', 'LineWidth', 1.5, 'DisplayName', '真値');

% 推定軌跡
if ~isempty(pos_summary.positions)
    plot(pos_summary.positions(:, 1), pos_summary.positions(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', '推定値');
end

legend('Location', 'best');

% 速度誤差
subplot(2, 3, 2);
plot(vel_summary.timestamps, vel_summary.errors, 'b-', 'LineWidth', 1);
title('速度推定誤差');
xlabel('時間 [s]');
ylabel('誤差 [m/s]');
grid on;

% 姿勢誤差
subplot(2, 3, 3);
plot(att_summary.timestamps, rad2deg(att_summary.errors), 'r-', 'LineWidth', 1);
title('姿勢推定誤差 (Yaw)');
xlabel('時間 [s]');
ylabel('誤差 [deg]');
grid on;

% 位置誤差
subplot(2, 3, 4);
plot(pos_summary.timestamps, pos_summary.errors, 'g-', 'LineWidth', 1);
title('位置推定誤差');
xlabel('時間 [s]');
ylabel('誤差 [m]');
grid on;

% 誤差統計
subplot(2, 3, [5, 6]);
valid_vel = vel_summary.errors(~isnan(vel_summary.errors));
valid_att = att_summary.errors(~isnan(att_summary.errors));
valid_pos = pos_summary.errors(~isnan(pos_summary.errors));

error_data = [valid_vel; rad2deg(valid_att); valid_pos];
error_labels = [repmat({'速度 [m/s]'}, length(valid_vel), 1); ...
                repmat({'姿勢 [deg]'}, length(valid_att), 1); ...
                repmat({'位置 [m]'}, length(valid_pos), 1)];

if ~isempty(error_data)
    boxplot(error_data, error_labels);
    title('推定誤差分布');
    ylabel('誤差');
end

end