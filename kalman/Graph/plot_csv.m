function plot_csv(filePath)
% PLOT_CSV  estimation.csv を読み可視化する簡易関数
if ~exist(filePath,'file')
    error('File not found: %s', filePath);
end
T = readtable(filePath);

time = T.time;

figure;
subplot(3,1,1);
plot(time, T.px, time, T.py, time, T.pz);
legend('px','py','pz'); title('Position [m]'); grid on;

subplot(3,1,2);
plot(time, T.vx, time, T.vy, time, T.vz);
legend('vx','vy','vz'); title('Velocity [m/s]'); grid on;

subplot(3,1,3);
plot(time, T.roll, time, T.pitch, time, T.yaw);
legend('roll','pitch','yaw'); title('Attitude [rad]'); grid on;
end
