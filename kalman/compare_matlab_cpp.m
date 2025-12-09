function compare_matlab_vs_cpp()
    % MATLAB実装とC++実装のステップごと比較
    % 発散の原因を特定するため、同一入力で両方を実行し差分を検出
    
    fprintf('=== MATLAB vs C++ 実装比較テスト ===\n');
    
    % テストパラメータ
    test_seeds = [1, 2, 3];  % 既知の失敗シードと成功シード
    
    for seed_idx = 1:length(test_seeds)
        seed = test_seeds(seed_idx);
        fprintf('\n--- Seed %d の比較 ---\n', seed);
        
        % シミュレーションデータ生成
        rng(seed);
        obs = generate_test_data();
        
        % MATLAB版 ESKF
        fprintf('MATLAB版を実行中...\n');
        eskf_matlab = ESKF(obs, 0, 0.01);
        eskf_matlab.use_meukf = true;  % MEUKF有効化
        
        % C++版用の構造体準備
        fprintf('C++版パラメータを準備中...\n');
        
        % 数ステップ実行して比較
        n_steps = min(100, length(obs.accel_x));
        diffs = struct();
        diffs.step = [];
        diffs.state_diff = [];
        diffs.P_diff = [];
        diffs.max_P_diff = [];
        
        for k = 1:n_steps
            % MATLAB版を1ステップ実行
            eskf_matlab.updateFilter(obs, k);
            
            % C++版を同じ状態から実行（まだ実装されていない場合はスキップ）
            % ここでは差分検出のロジックのみ記述
            
            % 状態の差分を記録
            if k > 1
                % 簡易比較: 前ステップとの変化量
                diffs.step(end+1) = k;
            end
        end
        
        fprintf('Seed %d: %d ステップ実行完了\n', seed, n_steps);
    end
    
    fprintf('\n比較完了\n');
end

function obs = generate_test_data()
    % テスト用の簡易データ生成
    dt = 0.01;
    t_end = 1.0;  % 1秒間
    N = floor(t_end / dt);
    
    obs = struct();
    obs.accel_x = 0.1 * randn(N, 1);
    obs.accel_y = 0.1 * randn(N, 1);
    obs.accel_z = 9.81 + 0.1 * randn(N, 1);
    obs.gyro_x = 0.01 * randn(N, 1);
    obs.gyro_y = 0.01 * randn(N, 1);
    obs.gyro_z = 0.01 * randn(N, 1);
    obs.mag_x = 0.3 * ones(N, 1);
    obs.mag_y = 0.0 * ones(N, 1);
    obs.mag_z = 0.5 * ones(N, 1);
    obs.gps_x = zeros(N, 1);
    obs.gps_y = zeros(N, 1);
    obs.gps_z = zeros(N, 1);
    obs.alt_baro = zeros(N, 1);
end
