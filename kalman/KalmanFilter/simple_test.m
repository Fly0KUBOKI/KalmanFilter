% simple_test - 基本的なテスト

fprintf('基本テスト開始\n');

% 1. CSVファイルの存在確認
csv_file = fullfile('..', '..', 'sim_data.csv');
fprintf('CSVファイルパス: %s\n', csv_file);
if exist(csv_file, 'file')
    fprintf('CSVファイル存在: OK\n');
else
    fprintf('CSVファイル存在: NG\n');
    return;
end

% 2. UKFクラステスト
try
    ukf = UKF_Calculator(3, 2);
    fprintf('UKFクラス作成: OK\n');
catch ME
    fprintf('UKFクラス作成エラー: %s\n', ME.message);
    return;
end

% 3. 簡単な計算テスト
x = randn(3, 1);
P = eye(3);
F = eye(3);
Q = eye(3) * 0.1;

try
    [x_pred, P_pred] = ukf.predict(x, P, F, Q, 0.1);
    fprintf('UKF予測計算: OK\n');
    fprintf('予測状態: [%.3f, %.3f, %.3f]\n', x_pred);
catch ME
    fprintf('UKF予測計算エラー: %s\n', ME.message);
    return;
end

fprintf('基本テスト完了\n');