% quick_check - check accel3_x and gyro3_z for outliers
current_dir = fileparts(mfilename('fullpath'));
logfile = fullfile(current_dir, 'quick_check.log');
if exist(logfile,'file'), delete(logfile); end

diary(logfile); diary on;

csv_file = fullfile(current_dir, '..', '..', 'sim_data.csv');
T = readtable(csv_file);

cols = {'accel3_x','accel3_y','accel3_z','gyro3_x','gyro3_y','gyro3_z'};
for k=1:length(cols)
    v = T.(cols{k});
    fprintf('%s: min=%.6g max=%.6g mean=%.6g std=%.6g\n', cols{k}, min(v), max(v), mean(v), std(v));
    p99 = prctile(v,99);
    p999 = prctile(v,99.9);
    fprintf('  99th=%.6g 99.9th=%.6g\n', p99, p999);
    fprintf('  count(|v|>10)=%d count(|v|>100)=%d\n', sum(abs(v)>10), sum(abs(v)>100));
end

% show some rows where gyro3_z large
idx_large = find(abs(T.gyro3_z) > 1);
fprintf('rows with |gyro3_z|>1: %d (show up to 10 rows)\n', length(idx_large));
for i=1:min(10,length(idx_large))
    r = idx_large(i);
    fprintf(' row %d: gyro3_z=%.6g accel3=[%.6g %.6g %.6g] gyro3=[%.6g %.6g %.6g]\n', r, T.gyro3_z(r), T.accel3_x(r), T.accel3_y(r), T.accel3_z(r), T.gyro3_x(r), T.gyro3_y(r), T.gyro3_z(r));
end

% show rows where accel3_x abs>10
idx_ax = find(abs(T.accel3_x) > 10);
fprintf('rows with |accel3_x|>10: %d (show up to 10 rows)\n', length(idx_ax));
for i=1:min(10,length(idx_ax))
    r = idx_ax(i);
    fprintf(' row %d: accel3=[%.6g %.6g %.6g] gyro3=[%.6g %.6g %.6g]\n', r, T.accel3_x(r), T.accel3_y(r), T.accel3_z(r), T.gyro3_x(r), T.gyro3_y(r), T.gyro3_z(r));
end

fprintf('done\n');
diary off;
save(fullfile(current_dir,'quick_check.mat'));
