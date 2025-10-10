function eskf_demo()
% Minimal demo for the ESKF implementation using sim_data.csv in repo root
root = fileparts(mfilename('fullpath'));
% explicit common locations to search (repo root, kalman, current)
repo_root = fullfile(root,'..','..');
cand = {
    fullfile(repo_root,'sim_data.csv'), ... % repo root
    fullfile(root,'..','sim_data.csv'), ... % kalman folder
    fullfile(root,'sim_data.csv'), ...      % eskf folder
};
csvfile = '';
for i=1:numel(cand)
    if exist(cand{i},'file')
        csvfile = cand{i}; break;
    end
end
if isempty(csvfile)
    msg = sprintf(['sim_data.csv not found. Searched:\n  %s\n  %s\n  %s\n', ...
        'Please place sim_data.csv in the repository root or kalman/ folder.'], cand{1}, cand{2}, cand{3});
    error(msg);
end
T = readtable(csvfile);
% build params
params.dt = 1/400; % 400Hz base
params.noise.imu_gyro = 0.002; params.noise.imu_accel = 0.05;
params.noise.gyro_bias_walk = 1e-5; params.noise.accel_bias_walk = 1e-4;
params.noise.mag3 = 0.1; params.noise.baro = 0.5;
params.noise.gps_pos = 3.0; params.noise.gps_vel = 0.5;
params.sensors.mag_field = [1;0;0];

% initial nominal
nominal.pos = [0;0;0]; nominal.vel=[0;0;0]; nominal.quat=[1;0;0;0]; nominal.bg=[0;0;0]; nominal.ba=[0;0;0];
P = eye(15) * 0.1;
N = height(T);
for k=1:N
    meas = struct();
    if ismember('gyro_x', T.Properties.VariableNames)
        meas.imu.gyro = [T.gyro_x(k); T.gyro_y(k); T.gyro_z(k)];
    end
    if ismember('accel_x', T.Properties.VariableNames)
        meas.imu.accel = [T.accel_x(k); T.accel_y(k); T.accel_z(k)];
    end
    if ismember('mag3_x', T.Properties.VariableNames)
        meas.mag3 = [T.mag3_x(k); T.mag3_y(k); T.mag3_z(k)];
    end
    if ismember('baro', T.Properties.VariableNames)
        meas.baro = T.baro(k);
    end
    % GPS: build 3D vector even if CSV only contains 2D by filling missing
    % axes from current nominal state to avoid size mismatches.
    if ismember('gps_x', T.Properties.VariableNames) || ismember('gps_y', T.Properties.VariableNames)
        pos = nominal.pos(:); % default fill
        if ismember('gps_x', T.Properties.VariableNames), pos(1) = T.gps_x(k); end
        if ismember('gps_y', T.Properties.VariableNames), pos(2) = T.gps_y(k); end
        if ismember('gps_z', T.Properties.VariableNames), pos(3) = T.gps_z(k); end
        meas.gps = pos;
        % velocity (optional columns meas_vel_x/y/z)
        vel = nominal.vel(:);
        if ismember('meas_vel_x', T.Properties.VariableNames), vel(1) = T.meas_vel_x(k); end
        if ismember('meas_vel_y', T.Properties.VariableNames), vel(2) = T.meas_vel_y(k); end
        if ismember('meas_vel_z', T.Properties.VariableNames), vel(3) = T.meas_vel_z(k); end
        meas.vel = vel;
    end
    [nominal, P, ~] = eskf_filter_step(nominal, P, meas, params);
end
fprintf('Demo finished. Nominal pos: [%g %g %g]\n', nominal.pos);
end
