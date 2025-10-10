function test_components()
% TEST_COMPONENTS - 各コンポーネントの動作テスト

fprintf('=== コンポーネントテスト開始 ===\n');

try
    % パス設定
    current_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(current_dir, '..', 'GenerateData'));
    
    fprintf('1. パラメータ読み込みテスト...\n');
    params = config_params();
    fprintf('   OK - パラメータ読み込み成功\n');
    
    fprintf('2. CSVデータ読み込みテスト...\n');
    csv_file = fullfile(current_dir, '..', '..', 'sim_data.csv');
    [sensor_data, truth_data] = read_csv_data(csv_file);
    fprintf('   OK - CSVデータ読み込み成功 (%d行)\n', length(sensor_data.t));
    
    fprintf('3. UKFクラステスト...\n');
    ukf_vel = UKF_Calculator(6, 3);
    fprintf('   OK - UKF速度推定クラス作成成功\n');
    
    fprintf('4. 速度推定テスト（1ステップ）...\n');
    vel_result = velocity_estimator(sensor_data, params, ukf_vel, 1, []);
    fprintf('   OK - 速度推定成功\n');
    fprintf('   推定速度: [%.3f, %.3f, %.3f] m/s\n', vel_result.velocity);
    
    fprintf('5. 姿勢推定テスト（1ステップ）...\n');
    ukf_att = UKF_Calculator(6, 6);
    att_result = attitude_estimator(sensor_data, params, ukf_att, 1, [], vel_result);
    fprintf('   OK - 姿勢推定成功\n');
    fprintf('   推定姿勢: [%.3f, %.3f, %.3f] rad\n', att_result.attitude);
    
    fprintf('6. 位置推定テスト（1ステップ）...\n');
    ukf_pos = UKF_Calculator(6, 6);
    pos_result = position_estimator(sensor_data, params, ukf_pos, 1, [], {vel_result}, {att_result});
    fprintf('   OK - 位置推定成功\n');
    fprintf('   推定位置: [%.3f, %.3f, %.3f] m\n', pos_result.position);
    
    fprintf('=== すべてのテスト完了 ===\n');
    
catch ME
    fprintf('エラー発生: %s\n', ME.message);
    fprintf('スタックトレース:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (行 %d): %s\n', ME.stack(i).file, ME.stack(i).line, ME.stack(i).name);
    end
end

end