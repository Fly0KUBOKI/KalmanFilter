function diagnose_compiler_difference()
    % MinGW vs MSVC でのコンパイラ差異を診断
    % 中間状態を詳細にダンプして差分を特定
    
    fprintf('=== コンパイラ差異診断ツール ===\n\n');
    
    % 現在のコンパイラ情報を確認
    cc = mex.getCompilerConfigurations('C++', 'Selected');
    fprintf('現在のコンパイラ: %s\n', cc.Name);
    fprintf('バージョン: %s\n', cc.Version);
    fprintf('場所: %s\n\n', cc.Location);
    
    % MEXバイナリ情報
    mex_path = fullfile(pwd, 'kalman', 'cpp', 'bin', 'mex_hybrid_filter.mexw64');
    bin_dir = fullfile(pwd, 'kalman', 'cpp', 'bin');
    if exist(mex_path, 'file')
        info = dir(mex_path);
        fprintf('MEXバイナリ情報:\n');
        fprintf('  パス: %s\n', mex_path);
        fprintf('  サイズ: %d bytes (%.2f KB)\n', info.bytes, info.bytes/1024);
        fprintf('  更新日時: %s\n\n', info.date);
        
        % バイナリサイズによる判定
        if info.bytes > 500000
            fprintf('⚠️ バイナリが大きすぎます（デバッグビルドの可能性）\n');
        elseif info.bytes < 50000
            fprintf('⚠️ バイナリが小さすぎます\n');
        else
            fprintf('✅ バイナリサイズは正常範囲です\n');
        end
    else
        fprintf('❌ MEXバイナリが見つかりません: %s\n', mex_path);
        return;
    end

    % Ensure bin_dir is on path and MEX is visible
    if exist(bin_dir, 'dir')
        addpath(bin_dir);
        rehash path;
    end

    if exist('mex_hybrid_filter', 'file') ~= 3
        fprintf('❌ mex_hybrid_filter MEX not visible on path (exist~=3).\n');
        fprintf('   Checked path: %s\n', bin_dir);
        return;
    end
    
    fprintf('\n--- テスト開始 ---\n');
    
    % 固定シードでデータ生成
    seed = 42;
    rng(seed, 'twister');
    
    % データ読み込み
    data_path = fullfile(pwd, 'kalman', 'GenerateData', 'sensor_data.csv');
    if ~exist(data_path, 'file')
        fprintf('❌ センサーデータが見つかりません。先にgenerate_and_save_dataを実行してください。\n');
        return;
    end
    
    obs_data = readmatrix(data_path);
    n = size(obs_data, 1);
    
    % obs構造体の作成
    obs = struct();
    % sensor_data.csv columns:
    % 1 time, 2 accel_x, 3 accel_y, 4 accel_z,
    % 5 gyro_x, 6 gyro_y, 7 gyro_z, 8 mag_x, 9 mag_y, 10 mag_z,
    % 11 baro, 12 gps_lat, 13 gps_lon, 14 gps_alt
    obs.ax = single(obs_data(:,2));
    obs.ay = single(obs_data(:,3));
    obs.az = single(obs_data(:,4));
    obs.wx = single(obs_data(:,5));
    obs.wy = single(obs_data(:,6));
    obs.wz = single(obs_data(:,7));
    obs.mx = single(obs_data(:,8));
    obs.my = single(obs_data(:,9));
    obs.mz = single(obs_data(:,10));
    obs.pressure = single(obs_data(:,11));
    obs.lat = obs_data(:,12);
    obs.lon = obs_data(:,13);
    obs.alt = obs_data(:,14);
    
    static_time = 5.0;
    dt = 0.0025;
    
    % 初期化
    fprintf('\n【初期化フェーズ】\n');
    clear mex;  % MEXキャッシュをクリア
    
    try
        handle = mex_hybrid_filter('init', obs, static_time, dt);
        fprintf('✅ 初期化成功\n');
    catch ME
        fprintf('❌ 初期化失敗: %s\n', ME.message);
        return;
    end
    
    % 初期状態を取得
    state0 = mex_hybrid_filter('get_state', handle);
    fprintf('\n初期状態:\n');
    fprintf('  位置 p: [%.6f, %.6f, %.6f]\n', state0.p);
    fprintf('  速度 v: [%.6f, %.6f, %.6f]\n', state0.v);
    fprintf('  四元数 q: [%.6f, %.6f, %.6f, %.6f]\n', state0.q);
    fprintf('  加速度bias ba: [%.6f, %.6f, %.6f]\n', state0.ba);
    fprintf('  ジャイロbias bg: [%.6f, %.6f, %.6f]\n', state0.bg);
    fprintf('  オイラー角 [deg]: [%.3f, %.3f, %.3f]\n', state0.euler);
    
    % チェックポイント（特定のステップで中間状態を記録）
    checkpoints = [2001, 2010, 2050, 2100, 2500, 3000];
    states = struct();
    
    fprintf('\n【ステップ実行フェーズ】\n');
    static_steps = round(static_time / dt);
    
    for k = (static_steps+1):n
        mex_hybrid_filter('step', handle, obs, k);
        
        % チェックポイントで状態を記録
        if ismember(k, checkpoints)
            state = mex_hybrid_filter('get_state', handle);
            checkpoint_name = sprintf('step_%d', k);
            states.(checkpoint_name) = state;
            
            fprintf('\nステップ %d:\n', k);
            fprintf('  位置 p: [%.6f, %.6f, %.6f]\n', state.p);
            fprintf('  速度 v: [%.6f, %.6f, %.6f]\n', state.v);
            fprintf('  ジャイロbias bg: [%.6f, %.6f, %.6f]\n', state.bg);
            fprintf('  オイラー角 [deg]: [%.3f, %.3f, %.3f]\n', state.euler);
            
            % 異常値チェック
            if any(isnan(state.p)) || any(isinf(state.p))
                fprintf('  ⚠️ 位置に NaN/Inf が含まれています\n');
            end
            if max(abs(state.p)) > 1000
                fprintf('  ⚠️ 位置が異常に大きいです: %.2f m\n', max(abs(state.p)));
            end
            if all(abs(state.bg) < 1e-10)
                fprintf('  ⚠️ ジャイロbiasがゼロのままです（更新されていない可能性）\n');
            end
        end
    end
    
    % 最終状態
    fprintf('\n【最終状態】\n');
    final_state = mex_hybrid_filter('get_state', handle);
    fprintf('  位置 p: [%.6f, %.6f, %.6f]\n', final_state.p);
    fprintf('  速度 v: [%.6f, %.6f, %.6f]\n', final_state.v);
    fprintf('  四元数 q: [%.6f, %.6f, %.6f, %.6f]\n', final_state.q);
    fprintf('  加速度bias ba: [%.6f, %.6f, %.6f]\n', final_state.ba);
    fprintf('  ジャイロbias bg: [%.6f, %.6f, %.6f]\n', final_state.bg);
    fprintf('  オイラー角 [deg]: [%.3f, %.3f, %.3f]\n', final_state.euler);
    
    % 位置のRMSE計算
    pos_rmse = sqrt(mean(final_state.p.^2));
    fprintf('\n  位置RMSE: %.6f m\n', pos_rmse);
    
    if pos_rmse > 100
        fprintf('  ❌ 推定が大きく失敗しています\n');
    elseif pos_rmse > 10
        fprintf('  ⚠️ 推定精度が低いです\n');
    else
        fprintf('  ✅ 推定精度は許容範囲です\n');
    end
    
    % 結果を保存
    % Ensure Results directory exists under kalman/
    results_dir = fullfile(pwd, 'kalman', 'Results');
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
    result_file = fullfile(results_dir, sprintf('compiler_diagnosis_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
    save(result_file, 'cc', 'info', 'state0', 'states', 'final_state', 'checkpoints');
    fprintf('\n診断結果を保存: %s\n', result_file);
    
    % クリーンアップ
    mex_hybrid_filter('free', handle);
    
    fprintf('\n--- 診断完了 ---\n');
    fprintf('次のステップ:\n');
    fprintf('1. MSVCとMinGWの両方で build_mex() を実行\n');
    fprintf('2. 両方で diagnose_compiler_difference() を実行\n');
    fprintf('3. 出力結果（特にジャイロbias）を比較\n');
end
