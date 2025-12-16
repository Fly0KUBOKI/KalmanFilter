% compare_phase1.m
% 比較: MATLAB 実装 vs mex_sensor_filter

% Determine test folder robustly (works when run via -r)
tests_dir = fileparts(mfilename('fullpath'));
if isempty(tests_dir)
    tests_dir = pwd;
end
% project root expected at two levels up from tests_dir (kalman)
project_root = fileparts(fileparts(tests_dir));
addpath(project_root);
% Ensure KF utils are on path for SensorAccelFilter / SensorGyroFilter
addpath(fullfile(project_root,'KF','Utils'));

results_file = fullfile(tests_dir,'compare_phase1_results.txt');
fid = fopen(results_file,'w');

try
    % サンプルデータ
    a_meas = [0.1; 0.0; 9.8];
    a_expected = [0;0;9.8];
    w_meas = [0.01; -0.02; 0.005];
    dt = 1/200;
    cutoff = 30;

    % MATLAB フィルタ（インスタンス）
    accel_filter = SensorAccelFilter(struct('ema_alpha',0.3,'history_size',50,'gravity_range',[8,11]));
    gyro_filter = SensorGyroFilter(struct('drift_learning_rate',1e-4,'history_size',50));

    % MEX があるか確認
    mex_available = (exist('mex_sensor_filter','file') == 3);
    fprintf(fid, 'mex_available=%d\n', double(mex_available));

    % MEX リセットと reset_zero テスト
    if mex_available
        try
            mex_sensor_filter('reset');
            fprintf(fid,'mex reset ok\n');
            mex_sensor_filter('reset_zero');
            fprintf(fid,'mex reset_zero ok\n');
        catch e
            fprintf(fid,'mex reset error: %s\n', e.message);
        end
    end

    % 加速度比較
    [a_mat, is_out_mat] = accel_filter.apply(a_meas, a_expected);
    if mex_available
        try
            [a_mex, is_out_mex] = mex_sensor_filter('accel', a_meas, a_expected);
        catch e
            a_mex = nan(3,1);
            is_out_mex = false;
            fprintf(fid,'mex accel error: %s\n', e.message);
        end
    else
        a_mex = nan(3,1);
        is_out_mex = false;
    end
    diff_a = a_mat - a_mex;
    fprintf(fid, 'accel_diff_norm=%.12g\n', norm(diff_a));
    fprintf(fid, 'accel_mat=%s\n', mat2str(a_mat',6));
    fprintf(fid, 'accel_mex=%s\n', mat2str(a_mex',6));

    % ジャイロ比較
    [w_mat, ~] = gyro_filter.apply(w_meas);
    if mex_available
        try
            w_mex = mex_sensor_filter('gyro', w_meas, dt, cutoff);
        catch e
            w_mex = nan(3,1);
            fprintf(fid,'mex gyro error: %s\n', e.message);
        end
    else
        w_mex = nan(3,1);
    end
    diff_w = w_mat - w_mex;
    fprintf(fid, 'gyro_diff_norm=%.12g\n', norm(diff_w));
    fprintf(fid, 'gyro_mat=%s\n', mat2str(w_mat',6));
    fprintf(fid, 'gyro_mex=%s\n', mat2str(w_mex',6));

    status = 0;
catch e
    fprintf(fid, 'ERROR: %s\n', getReport(e));
    status = 1;
end
fclose(fid);
exit(status);
