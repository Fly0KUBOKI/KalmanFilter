% test_phase1_fix.m
% Phase 1修正後のテスト

fprintf('=== Phase 1修正後テスト ===\n');
fprintf('mex_matlab_helpersのget_field修正をテスト\n\n');

% パス設定
addpath(pwd);
addpath(fullfile(pwd, 'cpp', 'bin'));
addpath(fullfile(pwd, 'ESKF'));
addpath(fullfile(pwd, 'GenerateData'));
addpath(fullfile(pwd, 'Graph'));

% ビルド
fprintf('1. mex_matlab_helpersをビルド中...\n');
try
    cd('cpp/build');
    build_mex({'mex_matlab_helpers'});
    cd('../..');
    fprintf('   ビルド成功\n');
catch ME
    cd('../..');
    fprintf('   ビルドエラー: %s\n', ME.message);
    rethrow(ME);
end

% テスト実行
fprintf('\n2. run_batch_10setsを実行中...\n');
try
    run_batch_10sets(false);
    fprintf('\n=== Phase 1修正後テスト完了 ===\n');
catch ME
    fprintf('\n=== Phase 1修正後テスト失敗 ===\n');
    fprintf('エラー: %s\n', ME.message);
    rethrow(ME);
end

