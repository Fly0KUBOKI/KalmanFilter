% quick_test_cpp.m
% 簡易版C++テスト - 各機能を個別にテスト

fprintf('Quick C++ Test\n\n');

% Resolve script root robustly in case MATLAB cwd differs
root = fileparts(which('quick_test_cpp'));
if isempty(root)
    root = pwd;
end

% テスト設定を配列で定義
tests = {
    'All MATLAB', false, false, false, false;
    'Accel C++',  true,  false, false, false;
    'Mag C++',    false, true,  false, false;
    'GPS C++',    false, false, true,  false;
    'Baro C++',   false, false, false, true;
    'All C++',    true,  true,  true,  true;
};

results = cell(size(tests, 1), 5);

for i = 1:size(tests, 1)
    test_name = tests{i, 1};
    use_accel = tests{i, 2};
    use_mag = tests{i, 3};
    use_gps = tests{i, 4};
    use_baro = tests{i, 5};
    
    fprintf('Test %d/%d: %s\n', i, size(tests, 1), test_name);
    fprintf('  Flags: accel=%d, mag=%d, gps=%d, baro=%d\n', ...
        use_accel, use_mag, use_gps, use_baro);
    
    % ESKF.mを編集
    edit_eskf_flags(use_accel, use_mag, use_gps, use_baro);
    
    % ESKFクラスのみクリア（他の変数は保持）
    clear ESKF;
    
    % シミュレーション実行
    try
        run_simulation(42, false);
        
        % 結果読み込み（スクリプトのルートから絶対パスで指定）
        est_file = fullfile(root, 'Results', 'estimation.csv');
        truth_file = fullfile(root, 'GenerateData', 'truth_data.csv');
        est = readtable(est_file);
        truth = readtable(truth_file);
        
        % 初期化期間スキップ
        init_samples = 2000;
        est = est(init_samples+1:end, :);
        truth = truth(init_samples+1:end, :);
        
        % RMSE計算
        pos_error = sqrt(mean((est.px - truth.x).^2 + (est.py - truth.y).^2 + (est.pz - truth.z).^2));
        roll_error = sqrt(mean(angdiff(deg2rad(est.roll), deg2rad(truth.roll)).^2)) * 180/pi;
        pitch_error = sqrt(mean(angdiff(deg2rad(est.pitch), deg2rad(truth.pitch)).^2)) * 180/pi;
        yaw_error = sqrt(mean(angdiff(deg2rad(est.yaw), deg2rad(truth.yaw)).^2)) * 180/pi;
        
        results{i, 1} = test_name;
        results{i, 2} = pos_error;
        results{i, 3} = roll_error;
        results{i, 4} = pitch_error;
        results{i, 5} = yaw_error;
        
        fprintf('  Position RMSE: %.4f m\n', pos_error);
        fprintf('  Attitude RMSE: Roll=%.4f, Pitch=%.4f, Yaw=%.4f deg\n\n', ...
            roll_error, pitch_error, yaw_error);
    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        fprintf('  Stack trace:\n');
        for k = 1:length(ME.stack)
            fprintf('    %s (line %d)\n', ME.stack(k).name, ME.stack(k).line);
        end
        fprintf('\n');
        results{i, 1} = test_name;
        results{i, 2} = NaN;
        results{i, 3} = NaN;
        results{i, 4} = NaN;
        results{i, 5} = NaN;
    end
end

% 結果表示
fprintf('\n===========================================\n');
fprintf('Summary\n');
fprintf('===========================================\n');
fprintf('%-15s | %10s | %10s | %10s | %10s\n', 'Test', 'Pos RMSE', 'Roll RMSE', 'Pitch RMSE', 'Yaw RMSE');
fprintf('----------------|------------|------------|------------|------------\n');
for i = 1:size(results, 1)
    fprintf('%-15s | %8.4f m | %8.4f° | %8.4f° | %8.4f°\n', ...
        results{i, 1}, results{i, 2}, results{i, 3}, results{i, 4}, results{i, 5});
end

% ログファイル出力
log_file = fullfile(root, 'Results', 'quick_test_cpp_log.txt');
fid = fopen(log_file, 'w');
fprintf(fid, 'Quick C++ Test Results\n');
fprintf(fid, 'Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, '===========================================\n');
fprintf(fid, 'Summary\n');
fprintf(fid, '===========================================\n');
fprintf(fid, '%-15s | %10s | %10s | %10s | %10s\n', 'Test', 'Pos RMSE', 'Roll RMSE', 'Pitch RMSE', 'Yaw RMSE');
fprintf(fid, '----------------|------------|------------|------------|------------\n');
for i = 1:size(results, 1)
    fprintf(fid, '%-15s | %8.4f m | %8.4f° | %8.4f° | %8.4f°\n', ...
        results{i, 1}, results{i, 2}, results{i, 3}, results{i, 4}, results{i, 5});
end
fclose(fid);
fprintf('\nログを保存しました: %s\n', log_file);

% 元に戻す
edit_eskf_flags(false, false, false, false);

function edit_eskf_flags(use_accel, use_mag, use_gps, use_baro)
    % Resolve script root robustly in case MATLAB cwd differs
    root = fileparts(which('quick_test_cpp'));
    if isempty(root)
        % fallback to file's relative location
        root = pwd;
    end
    filename = fullfile(root, 'ESKF', 'ESKF.m');
    content = fileread(filename);
    
    content = regexprep(content, 'obj\.use_cpp_accel = (true|false);', ...
        sprintf('obj.use_cpp_accel = %s;', mat2str(use_accel)));
    content = regexprep(content, 'obj\.use_cpp_mag = (true|false);', ...
        sprintf('obj.use_cpp_mag = %s;', mat2str(use_mag)));
    content = regexprep(content, 'obj\.use_cpp_gps = (true|false);', ...
        sprintf('obj.use_cpp_gps = %s;', mat2str(use_gps)));
    content = regexprep(content, 'obj\.use_cpp_baro = (true|false);', ...
        sprintf('obj.use_cpp_baro = %s;', mat2str(use_baro)));
    
    fid = fopen(filename, 'w');
    fprintf(fid, '%s', content);
    fclose(fid);
end
