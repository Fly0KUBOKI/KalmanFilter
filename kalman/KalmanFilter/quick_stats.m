% quick_stats - check sim_data.csv columns and sample values
current_dir = fileparts(mfilename('fullpath'));
logfile = fullfile(current_dir, 'quick_stats.log');
if exist(logfile,'file'), delete(logfile); end
diary(logfile); diary on;
csv_file = fullfile(current_dir, '..', '..', 'sim_data.csv');
T = readtable(csv_file);
fprintf('Header columns:\n');
disp(T.Properties.VariableNames');

fprintf('\nFirst row sample values for accel/gyro/mag/gps/baro:\n');
fprintf('accel3: %.6g %.6g %.6g\n', T.accel3_x(1), T.accel3_y(1), T.accel3_z(1));
fprintf('gyro3:  %.6g %.6g %.6g\n', T.gyro3_x(1), T.gyro3_y(1), T.gyro3_z(1));
fprintf('mag3:   %.6g %.6g %.6g\n', T.mag3_x(1), T.mag3_y(1), T.mag3_z(1));
fprintf('gps:    %.6g %.6g\n', T.gps_x(1), T.gps_y(1));
fprintf('baro:   %.6g\n', T.baro(1));

N=1000;
N = min(height(T), N);
fprintf('\nStd dev (first %d rows):\n', N);
fprintf('accel std: %.6g %.6g %.6g\n', std(T.accel3_x(1:N)), std(T.accel3_y(1:N)), std(T.accel3_z(1:N)));
fprintf('gyro std:  %.6g %.6g %.6g\n', std(T.gyro3_x(1:N)), std(T.gyro3_y(1:N)), std(T.gyro3_z(1:N)));
fprintf('mag std:   %.6g %.6g %.6g\n', std(T.mag3_x(1:N)), std(T.mag3_y(1:N)), std(T.mag3_z(1:N)));
fprintf('gps std:   %.6g %.6g\n', std(T.gps_x(1:N)), std(T.gps_y(1:N)));

save(fullfile(current_dir,'quick_stats.mat'));
fprintf('done\n');
diary off;
