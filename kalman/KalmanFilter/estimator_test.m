% estimator_test - 各推定器の個別テスト

fprintf('推定器テスト開始\n');

% パス設定とデータ読み込み
addpath(fullfile('..', 'GenerateData'));
csv_file = fullfile('..', '..', 'sim_data.csv');
[sensor_data, truth_data] = read_csv_data(csv_file);
params = config_params();

fprintf('データ読み込み完了\n');

try
    % 1. 速度推定器テスト
    fprintf('1. 速度推定器テスト...\n');
    ukf_vel = UKF_Calculator(6, 3);
    vel_result = velocity_estimator(sensor_data, params, ukf_vel, 1, []);
    fprintf('   速度推定成功: [%.3f, %.3f, %.3f] m/s\n', vel_result.velocity);
    
    % 2. 姿勢推定器テスト
    fprintf('2. 姿勢推定器テスト...\n');
    ukf_att = UKF_Calculator(6, 6);
    att_result = attitude_estimator(sensor_data, params, ukf_att, 4, [], vel_result);
    fprintf('   姿勢推定成功: [%.3f, %.3f, %.3f] rad\n', att_result.attitude);
    
    % 3. 位置推定器テスト
    fprintf('3. 位置推定器テスト...\n');
    ukf_pos = UKF_Calculator(6, 6);
    pos_result = position_estimator(sensor_data, params, ukf_pos, 40, [], {vel_result}, {att_result});
    fprintf('   位置推定成功: [%.3f, %.3f, %.3f] m\n', pos_result.position);
    
    % 4. 複数ステップテスト（小規模）
    fprintf('4. 複数ステップテスト（10ステップ）...\n');
    
    vel_results = cell(10, 1);
    prev_vel_state = [];
    
    for i = 1:10
        vel_results{i} = velocity_estimator(sensor_data, params, ukf_vel, i, prev_vel_state);
        prev_vel_state = vel_results{i};
        if mod(i, 5) == 0
            fprintf('   ステップ %d: 速度 [%.3f, %.3f, %.3f]\n', i, vel_results{i}.velocity);
        end
    end
    
    fprintf('推定器テスト完了 - すべて正常\n');
    
catch ME
    fprintf('エラー: %s\n', ME.message);
    fprintf('場所: %s (行 %d)\n', ME.stack(1).name, ME.stack(1).line);
    if length(ME.stack) > 1
        for i = 2:min(3, length(ME.stack))
            fprintf('  -> %s (行 %d)\n', ME.stack(i).name, ME.stack(i).line);
        end
    end
end