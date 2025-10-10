% debug_run - 短時間デバッグ実行 (最初の200サンプル)
fprintf('debug_run start\n');
% determine current_dir and start diary to capture all output to a log file
current_dir = fileparts(mfilename('fullpath'));
logfile = fullfile(current_dir, 'debug_run.log');
if exist(logfile,'file')
    delete(logfile);
end
diary(logfile);
diary on;
genpath_dir = fullfile(current_dir, '..', 'GenerateData');
fprintf('current_dir=%s\n', current_dir);
fprintf('adding path: %s\n', genpath_dir);
if exist(genpath_dir, 'dir')
    addpath(genpath_dir);
else
    fprintf('WARNING: GenerateData directory not found: %s\n', genpath_dir);
end

csv_file = fullfile(current_dir, '..', '..', 'sim_data.csv');
fprintf('csv_file path: %s\n', csv_file);
if exist(csv_file, 'file')
    [sensor_data, truth_data] = read_csv_data(csv_file);
    fprintf('read_csv_data succeeded, rows=%d\n', length(sensor_data.t));
else
    error('CSV file not found at: %s', csv_file);
end

% --- quick sensor stats to detect unit mismatches ---
Ncheck = min(1000, length(sensor_data.t));
try
    dataTbl = readtable(csv_file);
    fprintf('Sensor std (first %d rows): accel [%.3g %.3g %.3g], gyro [%.3g %.3g %.3g], gps [%.3g %.3g]\n', Ncheck, ...
        std(dataTbl.accel3_x(1:Ncheck)), std(dataTbl.accel3_y(1:Ncheck)), std(dataTbl.accel3_z(1:Ncheck)), ...
        std(dataTbl.gyro3_x(1:Ncheck)), std(dataTbl.gyro3_y(1:Ncheck)), std(dataTbl.gyro3_z(1:Ncheck)), ...
        std(dataTbl.gps_x(1:Ncheck)), std(dataTbl.gps_y(1:Ncheck)));
catch
    fprintf('Warning: could not compute quick stds from CSV table\n');
end

params = config_params();
% print some params for debugging
fprintf('params.kf.process_noise_accel=%.3g\n', params.kf.process_noise_accel);
if isfield(params.kf,'P0')
    P0 = params.kf.P0;
    fprintf('params.kf.P0 diag (first 6): ');
    fprintf('%.3g ', diag(P0(1:min(end,6),1:min(end,6))));
    fprintf('\n');
end
if isfield(params.kf,'x0')
    fprintf('params.kf.x0 (first 6): ');
    fprintf('%.3g ', params.kf.x0(1:min(end,6))');
    fprintf('\n');
end

ukf_vel = UKF_Calculator(6,3);
ukf_att = UKF_Calculator(6,6);
ukf_pos = UKF_Calculator(6,6);

N = min(200, length(sensor_data.t));

prev_vel_state = [];
prev_att_state = [];
prev_pos_state = [];

vel_results = cell(N,1);
att_results = cell(ceil(N/4),1);
pos_results = cell(ceil(N/40),1);

vel_buffer = cell(4,1);
att_buffer = cell(10,1);

vel_buf_idx = 1; att_buf_idx = 1; att_idx = 1; pos_idx = 1;

for i=1:N
    dt = sensor_data.dt(i);
    % velocity
    latest_att = [];
    for j = length(att_results):-1:1
        if ~isempty(att_results{j})
            latest_att = att_results{j}; break; end
    end
    vel_results{i} = velocity_estimator(sensor_data, params, ukf_vel, i, prev_vel_state, latest_att);
    prev_vel_state = vel_results{i};
    vel_buffer{vel_buf_idx} = vel_results{i};
    vel_buf_idx = vel_buf_idx + 1; if vel_buf_idx>4, vel_buf_idx=1; end

    % attitude every 4
    if mod(i-1,4)==0
        vel_avg = calculate_velocity_average(vel_buffer);
        att_results{att_idx} = attitude_estimator(sensor_data, params, ukf_att, i, prev_att_state, vel_avg);
        prev_att_state = att_results{att_idx};
        att_buffer{att_buf_idx} = att_results{att_idx};
        att_buf_idx = att_buf_idx + 1; if att_buf_idx>10, att_buf_idx=1; end
        att_idx = att_idx + 1;
    end

    % position every 40
    if mod(i-1,40)==0
        vel_avg = calculate_velocity_average(vel_buffer);
        att_avg = calculate_attitude_average(att_buffer);
        pos_results{pos_idx} = position_estimator(sensor_data, params, ukf_pos, i, prev_pos_state, {vel_avg}, {att_avg});
        prev_pos_state = pos_results{pos_idx};
        pos_idx = pos_idx + 1;
    end

    % ログ: 各フィルタの x のノルムと P の最大固有値、イノベーションノルム
    if ~isempty(vel_results{i})
        vx = vel_results{i}.x; Pv = vel_results{i}.P;
        eigsP = eig(double(Pv));
        mineig = min(eigsP);
        maxeig = max(eigsP);
        innov_norm = norm(double(vel_results{i}.innovation));
        fprintf('i=%d VEL | |x|=%.3e minEigP=%.3e maxEigP=%.3e innov_norm=%.3e\n', i, norm(double(vx)), mineig, maxeig, innov_norm);
    end
    if mod(i-1,4)==0 && ~isempty(att_results{att_idx-1})
        xa = att_results{att_idx-1}.x; Pa = att_results{att_idx-1}.P;
        eigsPa = eig(double(Pa));
        fprintf('i=%d ATT | |x|=%.3e minEigP=%.3e maxEigP=%.3e innov_norm=%.3e\n', i, norm(double(xa)), min(eigsPa), max(eigsPa), norm(double(att_results{att_idx-1}.innovation)));
    end
    if mod(i-1,40)==0 && pos_idx>1 && ~isempty(pos_results{pos_idx-1})
        xp = pos_results{pos_idx-1}.x; Pp = pos_results{pos_idx-1}.P;
        eigsPp = eig(double(Pp));
        fprintf('i=%d POS | |x|=%.3e minEigP=%.3e maxEigP=%.3e innov_norm=%.3e\n', i, norm(double(xp)), min(eigsPp), max(eigsPp), norm(double(pos_results{pos_idx-1}.innovation)));
    end
end

save(fullfile(current_dir,'debug_results.mat'),'vel_results','att_results','pos_results');
fprintf('debug_run done\n');
