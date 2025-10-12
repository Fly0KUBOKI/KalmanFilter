function eskf_demo(csvfile)
% Minimal demo for the ESKF implementation.
% Usage: eskf_demo('C:/path/to/sim_data.csv')
% If csvfile is not provided, this function will NOT search the workspace and will
% error immediately (by design per user request).
root = fileparts(mfilename('fullpath'));
repo_root = fullfile(root,'..','..');
if nargin < 1 || isempty(csvfile)
    % default to repo root location but do not search other places
    csvfile = fullfile(repo_root,'sim_data.csv');
    if ~exist(csvfile,'file')
        error('sim_data.csv not found at %s. Provide path: eskf_demo(''path/to/sim_data.csv'')', csvfile);
    end
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
% reset adaptive R warmup containers if present
if ~isfield(params,'kf'), params.kf = struct(); end
params.kf.R_warmup_count = struct(); params.kf.R_warmup_sum = struct();

% prepare simple plotting
fig = figure('Name','ESKF Demo'); ax = axes(fig); hold(ax,'on'); grid(ax,'on');
h_true = plot(ax, NaN, NaN, '-k', 'DisplayName','True');
h_meas = plot(ax, NaN, NaN, '.r', 'DisplayName','Meas');
h_est = plot(ax, NaN, NaN, '-b', 'DisplayName','ESKF');
legend;

true_traj = [];
meas_traj = [];
est_traj = [];
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

    % logging for plot
    if ismember('x', T.Properties.VariableNames) && ismember('y', T.Properties.VariableNames)
        true_traj(end+1,:) = [T.x(k), T.y(k)];
    else
        true_traj(end+1,:) = nominal.pos(1:2)';
    end
    if isfield(meas,'gps') && numel(meas.gps) >= 2
        meas_traj(end+1,:) = meas.gps(1:2)';
    else
        meas_traj(end+1,:) = [NaN NaN];
    end
    est_traj(end+1,:) = nominal.pos(1:2)';

    % update plot occasionally to reduce overhead
    if mod(k,20)==0
        set(h_true, 'XData', true_traj(:,1), 'YData', true_traj(:,2));
        set(h_meas, 'XData', meas_traj(:,1), 'YData', meas_traj(:,2));
        set(h_est, 'XData', est_traj(:,1), 'YData', est_traj(:,2));
        drawnow limitrate;
    end
end
fprintf('Demo finished. Nominal pos: [%g %g %g]\n', nominal.pos);
end
