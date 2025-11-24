% 簡易的なオフセット計算
clc;

% CSVファイル読み込み
truth = readtable('GenerateData/truth_data.csv');
est = readtable('Results/estimation.csv');

% 運動期間（静止期間0.5s後）
motion_idx = 201:height(truth);

% 真値の範囲
fprintf('=== 真値の範囲 ===\n');
fprintf('X: [%.3f, %.3f] m\n', min(truth.x(motion_idx)), max(truth.x(motion_idx)));
fprintf('Y: [%.3f, %.3f] m\n', min(truth.y(motion_idx)), max(truth.y(motion_idx)));
fprintf('Z: [%.3f, %.3f] m\n', min(truth.z(motion_idx)), max(truth.z(motion_idx)));

% 推定値の範囲
fprintf('\n=== 推定値の範囲 ===\n');
fprintf('X: [%.3f, %.3f] m\n', min(est.px(motion_idx)), max(est.px(motion_idx)));
fprintf('Y: [%.3f, %.3f] m\n', min(est.py(motion_idx)), max(est.py(motion_idx)));
fprintf('Z: [%.3f, %.3f] m\n', min(est.pz(motion_idx)), max(est.pz(motion_idx)));

% オフセット計算（中央値）
offset_x = median(est.px(motion_idx)) - median(truth.x(motion_idx));
offset_y = median(est.py(motion_idx)) - median(truth.y(motion_idx));
offset_z = median(est.pz(motion_idx)) - median(truth.z(motion_idx));

fprintf('\n=== オフセット（中央値ベース） ===\n');
fprintf('Offset: [%.3f, %.3f, %.3f] m\n', offset_x, offset_y, offset_z);

% 最終10サンプルの比較
fprintf('\n=== 最終10サンプルの比較 ===\n');
for i = height(truth)-9:height(truth)
    err_x = est.px(i) - truth.x(i);
    err_y = est.py(i) - truth.y(i);
    err_z = est.pz(i) - truth.z(i);
    fprintf('t=%.2fs: Truth=[%6.2f,%6.2f,%6.2f], Est=[%6.2f,%6.2f,%6.2f], Err=[%6.2f,%6.2f,%6.2f]\n', ...
        truth.time(i), truth.x(i), truth.y(i), truth.z(i), ...
        est.px(i), est.py(i), est.pz(i), err_x, err_y, err_z);
end
