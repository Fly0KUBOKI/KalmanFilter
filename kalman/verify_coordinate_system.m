function verify_coordinate_system()
    % VERIFY_COORDINATE_SYSTEM  座標系と符号規約の検証
    %
    % 目的:
    %   真値の姿勢から加速度を逆算し、センサー測定値と一致するかを確認
    %   これにより、座標系の定義や符号規約が正しいかを検証する
    
    fprintf('========================================\n');
    fprintf('座標系・符号規約検証プログラム\n');
    fprintf('========================================\n\n');
    
    % パスの追加
    addpath('GenerateData');
    addpath('KF/Core');
    
    %% 1. データ読み込み
    fprintf('1. データを読み込み中...\n');
    truth_data = readmatrix('GenerateData/truth_data.csv');
    sensor_data = readmatrix('GenerateData/sensor_data.csv');
    
    time_truth = truth_data(:, 1);
    vel_x = truth_data(:, 5);
    vel_y = truth_data(:, 6);
    vel_z = truth_data(:, 7);
    roll_truth = truth_data(:, 8);
    pitch_truth = truth_data(:, 9);
    yaw_truth = truth_data(:, 10);
    
    ax_meas = sensor_data(:, 2);
    ay_meas = sensor_data(:, 3);
    az_meas = sensor_data(:, 4);
    
    N = length(time_truth);
    fprintf('   データ数: %d\n\n', N);
    
    %% 2. 真値データから並進加速度を計算
    fprintf('2. 真値速度から並進加速度を計算中...\n');
    dt = time_truth(2) - time_truth(1);
    acc_world_x = [0; diff(vel_x)/dt];
    acc_world_y = [0; diff(vel_y)/dt];
    acc_world_z = [0; diff(vel_z)/dt];
    
    fprintf('   サンプリング周期: %.4f 秒\n', dt);
    fprintf('   加速度統計（ワールド座標系）:\n');
    fprintf('     X: mean=%.3f, max=%.3f m/s²\n', mean(acc_world_x), max(abs(acc_world_x)));
    fprintf('     Y: mean=%.3f, max=%.3f m/s²\n', mean(acc_world_y), max(abs(acc_world_y)));
    fprintf('     Z: mean=%.3f, max=%.3f m/s²\n\n', mean(acc_world_z), max(abs(acc_world_z)));
    
    %% 3. 各ステップで検証
    fprintf('3. 真値姿勢から加速度を逆算し、測定値と比較中...\n');
    
    errors = zeros(N, 1);
    gravity_world = [0; 0; -9.81];  % 重力（下向き）
    
    % 最初の数ステップを詳細表示
    for k = 1:min(10, N)
        % 真値の姿勢からクォータニオンと回転行列を取得
        euler_deg = [roll_truth(k); pitch_truth(k); yaw_truth(k)];
        q_truth = quat_lib('euler_to_quat', euler_deg);
        R_truth = quat_lib('quat_to_rotm', q_truth);
        
        % ワールド座標系での並進加速度
        a_world = [acc_world_x(k); acc_world_y(k); acc_world_z(k)];
        
        % 比力（specific force） = 並進加速度 - 重力
        specific_force_world = a_world - gravity_world;
        
        % Body座標系に変換
        specific_force_body = R_truth' * specific_force_world;
        
        % センサー測定値
        a_meas = [ax_meas(k); ay_meas(k); az_meas(k)];
        
        % 誤差
        error_vec = specific_force_body - a_meas;
        errors(k) = norm(error_vec);
        
        if k <= 5
            fprintf('\n--- ステップ %d (t=%.3f s) ---\n', k, time_truth(k));
            fprintf('  真値姿勢: Roll=%.2f, Pitch=%.2f, Yaw=%.2f deg\n', ...
                euler_deg(1), euler_deg(2), euler_deg(3));
            fprintf('  並進加速度（ワールド）: [%.3f, %.3f, %.3f] m/s²\n', ...
                a_world(1), a_world(2), a_world(3));
            fprintf('  比力（ワールド）: [%.3f, %.3f, %.3f] m/s²\n', ...
                specific_force_world(1), specific_force_world(2), specific_force_world(3));
            fprintf('  比力（ボディ）推定: [%.3f, %.3f, %.3f] m/s²\n', ...
                specific_force_body(1), specific_force_body(2), specific_force_body(3));
            fprintf('  比力（ボディ）測定: [%.3f, %.3f, %.3f] m/s²\n', ...
                a_meas(1), a_meas(2), a_meas(3));
            fprintf('  誤差ベクトル: [%.3f, %.3f, %.3f] m/s²\n', ...
                error_vec(1), error_vec(2), error_vec(3));
            fprintf('  誤差ノルム: %.6f m/s²\n', errors(k));
        end
    end
    
    % 残りのステップは詳細表示なし
    for k = 11:N
        euler_deg = [roll_truth(k); pitch_truth(k); yaw_truth(k)];
        q_truth = quat_lib('euler_to_quat', euler_deg);
        R_truth = quat_lib('quat_to_rotm', q_truth);
        
        a_world = [acc_world_x(k); acc_world_y(k); acc_world_z(k)];
        specific_force_world = a_world - gravity_world;
        specific_force_body = R_truth' * specific_force_world;
        a_meas = [ax_meas(k); ay_meas(k); az_meas(k)];
        
        errors(k) = norm(specific_force_body - a_meas);
    end
    
    %% 4. 統計と判定
    fprintf('\n========================================\n');
    fprintf('検証結果\n');
    fprintf('========================================\n');
    
    error_rms = sqrt(mean(errors.^2));
    error_max = max(errors);
    error_mean = mean(errors);
    
    fprintf('加速度一致性誤差:\n');
    fprintf('  RMS:  %.6e m/s²\n', error_rms);
    fprintf('  Max:  %.6e m/s²\n', error_max);
    fprintf('  Mean: %.6e m/s²\n', error_mean);
    
    threshold = 1e-3;  % 1 mm/s²の許容誤差
    
    if error_rms < threshold
        fprintf('\n✓ [PASS] 座標系と符号規約は正しい\n');
        fprintf('  真値姿勢から計算した加速度は測定値と一致します\n');
    else
        fprintf('\n✗ [FAIL] 座標系または符号規約に問題あり\n');
        fprintf('  真値姿勢から計算した加速度が測定値と一致しません\n');
        fprintf('  推定誤差: %.3f m/s² (許容値: %.3f m/s²)\n', error_rms, threshold);
    end
    
    %% 5. グラフ表示
    fprintf('\n5. 誤差の時系列をプロット中...\n');
    
    figure('Position', [100, 100, 1200, 600]);
    
    subplot(2,1,1);
    plot(time_truth, errors, 'b-', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Error [m/s²]');
    title('Acceleration Reconstruction Error');
    grid on;
    
    subplot(2,1,2);
    semilogy(time_truth, errors, 'b-', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Error [m/s²] (log scale)');
    title('Acceleration Reconstruction Error (Log Scale)');
    grid on;
    
    fprintf('\nテスト完了\n');
end
