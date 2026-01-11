function test_compiler_consistency()
% TEST_COMPILER_CONSISTENCY - MinGW/MSVC間の数値一貫性を検証
%
% Usage:
%   test_compiler_consistency()
%
% 両コンパイラでビルドし、run_batch_10sets() を実行して結果を比較。
% 位置RMSE差分 < 0.01m、姿勢RMSE差分 < 0.01deg であれば PASS。
%
% 注意:
%   - 実行には約5分かかります（各コンパイラで2.5分程度）
%   - 事前に select_mex_compiler.m が利用可能である必要があります
%   - Results/ ディレクトリに一時ファイルが作成されます

    fprintf('=================================\n');
    fprintf('Compiler Consistency Test\n');
    fprintf('=================================\n\n');
    
    proj_root = fileparts(mfilename('fullpath'));
    results_dir = fullfile(proj_root, 'Results');
    
    % Resultsディレクトリの確認
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
    
    % 現在のコンパイラ設定を保存
    try
        cc = mex.getCompilerConfigurations('C++', 'Selected');
        original_compiler = cc.Name;
        fprintf('Current compiler: %s\n\n', original_compiler);
    catch
        warning('Could not detect current compiler');
        original_compiler = '';
    end
    
    %% MSVC でビルド & テスト
    fprintf('========================================\n');
    fprintf('Step 1: Building with MSVC\n');
    fprintf('========================================\n');
    
    try
        select_mex_compiler('msvc');
        cd(fullfile(proj_root, 'cpp', 'build'));
        build_mex();
        cd(proj_root);
        clear mex;
        
        fprintf('\nRunning batch test with MSVC...\n');
        run_batch_10sets();
        
        % 結果を保存
        msvc_summary = fullfile(results_dir, 'batch_10sets_summary.csv');
        msvc_backup = fullfile(results_dir, 'batch_msvc_temp.csv');
        if exist(msvc_summary, 'file')
            copyfile(msvc_summary, msvc_backup);
            fprintf('MSVC results saved to: %s\n', msvc_backup);
        else
            error('MSVC batch results not found');
        end
        
    catch ME
        fprintf('\n❌ MSVC build/test failed: %s\n', ME.message);
        return;
    end
    
    %% MinGW でビルド & テスト
    fprintf('\n========================================\n');
    fprintf('Step 2: Building with MinGW\n');
    fprintf('========================================\n');
    
    try
        select_mex_compiler('mingw');
        cd(fullfile(proj_root, 'cpp', 'build'));
        build_mex();
        cd(proj_root);
        clear mex;
        
        fprintf('\nRunning batch test with MinGW...\n');
        run_batch_10sets();
        
        % 結果を保存
        mingw_summary = fullfile(results_dir, 'batch_10sets_summary.csv');
        mingw_backup = fullfile(results_dir, 'batch_mingw_temp.csv');
        if exist(mingw_summary, 'file')
            copyfile(mingw_summary, mingw_backup);
            fprintf('MinGW results saved to: %s\n', mingw_backup);
        else
            error('MinGW batch results not found');
        end
        
    catch ME
        fprintf('\n❌ MinGW build/test failed: %s\n', ME.message);
        return;
    end
    
    %% 結果比較
    fprintf('\n========================================\n');
    fprintf('Step 3: Comparing Results\n');
    fprintf('========================================\n');
    
    try
        % CSV読み込み
        msvc_results = readtable(msvc_backup);
        mingw_results = readtable(mingw_backup);
        
        % 差分計算
        diff_pos_x = max(abs(msvc_results.PosX_RMSE_m - mingw_results.PosX_RMSE_m));
        diff_pos_y = max(abs(msvc_results.PosY_RMSE_m - mingw_results.PosY_RMSE_m));
        diff_pos_z = max(abs(msvc_results.PosZ_RMSE_m - mingw_results.PosZ_RMSE_m));
        diff_roll = max(abs(msvc_results.Roll_RMSE_deg - mingw_results.Roll_RMSE_deg));
        diff_pitch = max(abs(msvc_results.Pitch_RMSE_deg - mingw_results.Pitch_RMSE_deg));
        diff_yaw = max(abs(msvc_results.Yaw_RMSE_deg - mingw_results.Yaw_RMSE_deg));
        
        % 結果表示
        fprintf('\n--- RMSE Differences (MSVC vs MinGW) ---\n');
        fprintf('Position X: %.6f m (threshold: 0.01 m)\n', diff_pos_x);
        fprintf('Position Y: %.6f m (threshold: 0.01 m)\n', diff_pos_y);
        fprintf('Position Z: %.6f m (threshold: 0.01 m)\n', diff_pos_z);
        fprintf('Roll:       %.6f deg (threshold: 0.01 deg)\n', diff_roll);
        fprintf('Pitch:      %.6f deg (threshold: 0.01 deg)\n', diff_pitch);
        fprintf('Yaw:        %.6f deg (threshold: 0.01 deg)\n', diff_yaw);
        
        % 統計情報
        fprintf('\n--- Statistical Summary ---\n');
        fprintf('MSVC  - Position RMSE: Mean=%.4f, Std=%.4f m\n', ...
            mean(msvc_results.PosX_RMSE_m), std(msvc_results.PosX_RMSE_m));
        fprintf('MinGW - Position RMSE: Mean=%.4f, Std=%.4f m\n', ...
            mean(mingw_results.PosX_RMSE_m), std(mingw_results.PosX_RMSE_m));
        
        % 判定
        fprintf('\n========================================\n');
        fprintf('Test Result\n');
        fprintf('========================================\n');
        
        threshold_pos = 0.01;  % 1cm
        threshold_att = 0.01;  % 0.01 deg
        
        if diff_pos_x < threshold_pos && diff_pos_y < threshold_pos && diff_pos_z < threshold_pos && ...
           diff_roll < threshold_att && diff_pitch < threshold_att && diff_yaw < threshold_att
            fprintf('✅ PASS: Compiler consistency check passed\n');
            fprintf('   All differences are within tolerance.\n');
            passed = true;
        else
            fprintf('❌ FAIL: Compiler consistency check failed\n');
            if diff_pos_x >= threshold_pos || diff_pos_y >= threshold_pos || diff_pos_z >= threshold_pos
                fprintf('   Position RMSE difference exceeds tolerance\n');
            end
            if diff_roll >= threshold_att || diff_pitch >= threshold_att || diff_yaw >= threshold_att
                fprintf('   Attitude RMSE difference exceeds tolerance\n');
            end
            passed = false;
        end
        
        fprintf('\n');
        
    catch ME
        fprintf('\n❌ Results comparison failed: %s\n', ME.message);
        passed = false;
    end
    
    %% クリーンアップ
    % 一時ファイルを削除
    if exist(msvc_backup, 'file')
        delete(msvc_backup);
    end
    if exist(mingw_backup, 'file')
        delete(mingw_backup);
    end
    
    % 元のコンパイラに戻す（可能であれば）
    if ~isempty(original_compiler)
        try
            if contains(original_compiler, 'MinGW', 'IgnoreCase', true)
                select_mex_compiler('mingw');
            elseif contains(original_compiler, 'Microsoft', 'IgnoreCase', true) || ...
                   contains(original_compiler, 'Visual', 'IgnoreCase', true)
                select_mex_compiler('msvc');
            end
            fprintf('Restored compiler to: %s\n', original_compiler);
        catch
            warning('Could not restore original compiler');
        end
    end
    
    % 最終ビルド（元のコンパイラで）
    try
        cd(fullfile(proj_root, 'cpp', 'build'));
        build_mex();
        clear mex;
        cd(proj_root);
    catch
        warning('Could not rebuild with original compiler');
    end
    
    fprintf('\n========================================\n');
    fprintf('Test completed\n');
    fprintf('========================================\n');
    
    if ~passed
        error('Compiler consistency test failed');
    end
end
