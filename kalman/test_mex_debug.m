% test_mex_debug.m
% MEX関数の動作をデバッグ

% テストデータ作成
obs.ax = rand(100, 1);
obs.ay = rand(100, 1);
obs.az = rand(100, 1);

fprintf('=== MEX関数デバッグテスト ===\n\n');

% 単一インデックステスト
fprintf('1. 単一インデックステスト (idx=50):\n');
try
    result1 = mex_matlab_helpers('get_field', obs, {'ax', 'accel_x'}, 50, 3);
    fprintf('   成功: result = [%.4f, %.4f, %.4f]\n', result1);
    fprintf('   期待値: [%.4f, %.4f, %.4f]\n', obs.ax(50), obs.ay(50), obs.az(50));
catch ME
    fprintf('   失敗: %s\n', ME.message);
end

% 配列インデックステスト（これはMEXでは処理できない）
fprintf('\n2. 配列インデックステスト (idx=1:10):\n');
fprintf('   注意: 配列インデックスはMEXでは処理できないため、MATLAB実装を使用\n');

