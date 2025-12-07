% check_progress.m - バッチ実行の進捗確認
fprintf('=== バッチ実行進捗確認 ===\n\n');

% ログファイル確認
log_file = 'Results/batch_10sets_log.txt';
if exist(log_file, 'file')
    fprintf('ログファイル: %s\n', log_file);
    
    % ファイルサイズ
    info = dir(log_file);
    fprintf('ファイルサイズ: %.2f KB\n', info.bytes/1024);
    fprintf('最終更新: %s\n\n', info.date);
    
    % 最後の50行を表示
    fid = fopen(log_file, 'r');
    lines = {};
    while ~feof(fid)
        lines{end+1} = fgetl(fid);
    end
    fclose(fid);
    
    start_line = max(1, length(lines) - 50);
    fprintf('--- 最新50行 ---\n');
    for i = start_line:length(lines)
        if ischar(lines{i})
            fprintf('%s\n', lines{i});
        end
    end
else
    fprintf('ログファイルが見つかりません: %s\n', log_file);
end

% 生成されたCSVファイル数を確認
fprintf('\n=== 生成済みファイル ===\n');
csv_files = dir('Results/estimation_*.csv');
fprintf('生成済みCSV: %d個\n', length(csv_files));
for i = 1:length(csv_files)
    fprintf('  %s (%.2f MB)\n', csv_files(i).name, csv_files(i).bytes/1024/1024);
end
