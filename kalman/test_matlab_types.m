% test_matlab_types.m
% MATLAB側が正しくfloat（single）を渡しているかテスト

function test_matlab_types()
    clc; clear; close all;
    
    proj_root = fileparts(mfilename('fullpath'));
    addpath(fullfile(proj_root, 'GenerateData'));
    addpath(fullfile(proj_root, 'cpp', 'bin'));
    
    fprintf('=== MATLAB側のデータ型テスト ===\n\n');
    
    % 1. CSVからデータを読み込み
    obs_file = fullfile(proj_root, 'GenerateData', 'sensor_data.csv');
    if ~exist(obs_file, 'file')
        error('sensor_data.csv not found: %s', obs_file);
    end
    
    fprintf('1. read_csvで読み込んだデータの型:\n');
    obs = read_csv(obs_file);
    
    % 各フィールドの型を確認
    fields_to_check = {'time', 'ax', 'ay', 'az', 'wx', 'wy', 'wz', ...
                       'mx', 'my', 'mz', 'pressure', 'lat', 'lon', 'alt'};
    
    for i = 1:length(fields_to_check)
        field = fields_to_check{i};
        if isfield(obs, field)
            data_type = class(obs.(field));
            fprintf('  obs.%s: %s (size: %s)\n', field, data_type, mat2str(size(obs.(field))));
            
            % GPSデータはdouble、その他はsingleであるべき
            if strcmp(field, 'lat') || strcmp(field, 'lon') || strcmp(field, 'alt')
                if ~strcmp(data_type, 'double')
                    fprintf('    ⚠️ 警告: GPSデータはdoubleであるべきですが、%sです\n', data_type);
                else
                    fprintf('    ✓ GPSデータは正しくdoubleです\n');
                end
            else
                if ~strcmp(data_type, 'single')
                    fprintf('    ⚠️ 警告: センサーデータはsingleであるべきですが、%sです\n', data_type);
                else
                    fprintf('    ✓ センサーデータは正しくsingleです\n');
                end
            end
        else
            fprintf('  obs.%s: フィールドが存在しません\n', field);
        end
    end
    
    fprintf('\n2. サンプルデータの値（最初の5要素）:\n');
    sample_idx = min(5, length(obs.time));
    for i = 1:length(fields_to_check)
        field = fields_to_check{i};
        if isfield(obs, field) && length(obs.(field)) >= sample_idx
            fprintf('  obs.%s(1:%d) = %s\n', field, sample_idx, mat2str(obs.(field)(1:sample_idx)));
        end
    end
    
    fprintf('\n3. MEX関数に渡す前の型チェック:\n');
    % mex_run_eskf('step', ...)に渡されるobsの型を確認
    k = 1;
    fprintf('  k = %d でのobs構造体の型:\n', k);
    for i = 1:length(fields_to_check)
        field = fields_to_check{i};
        if isfield(obs, field)
            data_type = class(obs.(field));
            if k <= length(obs.(field))
                value = obs.(field)(k);
                fprintf('    obs.%s(%d) = %g (type: %s)\n', field, k, value, data_type);
            end
        end
    end
    
    fprintf('\n4. MEX関数の初期化テスト:\n');
    try
        params = config_params();
        dt = mean(diff(obs.time));
        fprintf('  dt = %g\n', dt);
        fprintf('  params.static_time = %g\n', params.static_time);
        
        handle = mex_run_eskf('init', obs, params.static_time, dt);
        fprintf('  ✓ 初期化成功 (handle = %d)\n', handle);
        
        % 1ステップ実行を試みる
        fprintf('\n5. 1ステップ実行テスト:\n');
        static_samples = floor(params.static_time / dt);
        test_k = static_samples + 1;
        if test_k <= length(obs.time)
            fprintf('  k = %d でstepを実行...\n', test_k);
            mex_run_eskf('step', handle, obs, test_k);
            fprintf('  ✓ step実行成功\n');
        else
            fprintf('  ⚠️ テスト用のサンプルが不足しています\n');
        end
        
        % クリーンアップ
        mex_run_eskf('free', handle);
        fprintf('  ✓ クリーンアップ完了\n');
        
    catch ME
        fprintf('  ✗ エラー発生:\n');
        fprintf('    メッセージ: %s\n', ME.message);
        fprintf('    場所: %s (行 %d)\n', ME.stack(1).file, ME.stack(1).line);
        if length(ME.stack) > 1
            for i = 2:length(ME.stack)
                fprintf('            %s (行 %d)\n', ME.stack(i).file, ME.stack(i).line);
            end
        end
        rethrow(ME);
    end
    
    fprintf('\n=== テスト完了 ===\n');
end

