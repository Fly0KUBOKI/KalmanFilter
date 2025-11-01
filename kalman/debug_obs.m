% obs構造体の確認
clear; clc;

addpath('GenerateData');
obs = read_csv('GenerateData/sensor_data.csv');

fprintf('=== obs構造体のフィールド ===\n');
disp(fieldnames(obs));

fprintf('\n=== 最初の10サンプルの加速度データ ===\n');
for k = 1:10
    fprintf('k=%d: ax=%.6f, ay=%.6f, az=%.6f\n', k, obs.ax(k), obs.ay(k), obs.az(k));
end

fprintf('\n=== 静止期間（800サンプル）の統計 ===\n');
fprintf('ax: mean=%.6f, std=%.6f\n', mean(obs.ax(1:800)), std(obs.ax(1:800)));
fprintf('ay: mean=%.6f, std=%.6f\n', mean(obs.ay(1:800)), std(obs.ay(1:800)));
fprintf('az: mean=%.6f, std=%.6f\n', mean(obs.az(1:800)), std(obs.az(1:800)));
