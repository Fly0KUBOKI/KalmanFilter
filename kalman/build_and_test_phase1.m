% build_and_test_phase1.m
% Phase 1修正後のビルドとテスト

fprintf('=== Phase 1修正後ビルドとテスト ===\n\n');

% パス設定
proj_root = fileparts(mfilename('fullpath'));
addpath(proj_root);
addpath(fullfile(proj_root, 'cpp', 'bin'));
addpath(fullfile(proj_root, 'ESKF'));
addpath(fullfile(proj_root, 'GenerateData'));
addpath(fullfile(proj_root, 'Graph'));

% ビルド
fprintf('1. mex_matlab_helpersをビルド中...\n');
try
    build_dir = fullfile(proj_root, 'cpp', 'build');
    cd(build_dir);
    build_mex({'mex_matlab_helpers'});
    cd(proj_root);
    fprintf('   ビルド成功\n');
catch ME
    cd(proj_root);
    fprintf('   ビルドエラー: %s\n', ME.message);
    rethrow(ME);
end

% MEXをクリア
clear mex;

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

