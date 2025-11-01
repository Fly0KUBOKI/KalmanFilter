% 推定結果と真値を比較
clear; clc;

est = readtable('Results/estimation.csv');
truth = readtable('GenerateData/truth_data.csv');

% サンプル点
samples = [1, 801, 10001, 20001, 30001, 40000];
times = est.time(samples);

fprintf('時刻\t位置誤差\t速度誤差\t姿勢誤差\troll\tpitch\tyaw\n');
fprintf('----------------------------------------------------------------\n');

for i = 1:length(samples)
    k = samples(i);
    
    pos_err = norm([est.px(k)-truth.x(k), est.py(k)-truth.y(k), est.pz(k)-truth.z(k)]);
    vel_err = norm([est.vx(k)-truth.vx(k), est.vy(k)-truth.vy(k), est.vz(k)-truth.vz(k)]);
    att_err = norm([est.roll(k)-truth.roll(k), est.pitch(k)-truth.pitch(k), est.yaw(k)-truth.yaw(k)]);
    
    fprintf('%.2f\t%.4f\t%.4f\t%.4f\t%.2f\t%.2f\t%.2f\n', times(i), pos_err, vel_err, att_err, ...
        est.roll(k), est.pitch(k), est.yaw(k));
end

fprintf('\n最終結果（t=99.9975秒）:\n');
k = 40000;
fprintf('推定: p=[%.4f, %.4f, %.4f], v=[%.4f, %.4f, %.4f]\n', ...
    est.px(k), est.py(k), est.pz(k), est.vx(k), est.vy(k), est.vz(k));
fprintf('真値: p=[%.4f, %.4f, %.4f], v=[%.4f, %.4f, %.4f]\n', ...
    truth.x(k), truth.y(k), truth.z(k), truth.vx(k), truth.vy(k), truth.vz(k));
fprintf('誤差: Δp=[%.4f, %.4f, %.4f], Δv=[%.4f, %.4f, %.4f]\n', ...
    est.px(k)-truth.x(k), est.py(k)-truth.y(k), est.pz(k)-truth.z(k), ...
    est.vx(k)-truth.vx(k), est.vy(k)-truth.vy(k), est.vz(k)-truth.vz(k));
