% test_phase1.m
% Phase 1統合テスト用スクリプト
% kalmanディレクトリから実行

addpath(pwd);
addpath(fullfile(pwd, 'cpp', 'bin'));
addpath(fullfile(pwd, 'ESKF'));
addpath(fullfile(pwd, 'GenerateData'));
addpath(fullfile(pwd, 'Graph'));

fprintf('=== Phase 1統合テスト開始 ===\n');
fprintf('mex_matlab_helpersを使用してget_field_impl, has_field_implをテスト\n\n');

try
    run_batch_10sets(false);
    fprintf('\n=== Phase 1テスト完了 ===\n');
catch ME
    fprintf('\n=== Phase 1テスト失敗 ===\n');
    fprintf('エラー: %s\n', ME.message);
    fprintf('スタック:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).file, ME.stack(i).line);
    end
    rethrow(ME);
end

