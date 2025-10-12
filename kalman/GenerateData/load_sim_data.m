function [true_state_full, meas_full, csvN] = load_sim_data(csv_file)
% load_sim_data - read CSV once and pre-extract sensor fields into arrays
% [true_state_full, meas_full, csvN] = load_sim_data(csv_file)
% true_state_full: Mx4 array [x y vx vy]
% meas_full: struct with fields pos (Mx2), vel (Mx2), and optional accel3, gyro3, mag3, gps, baro, heading
% csvN: number of rows in CSV

% if CSV doesn't exist, try to generate it using generate_and_save_data
if ~exist(csv_file,'file')
    fprintf('CSV not found: %s\nGenerating simulated data at that path...\n', csv_file);
    try
        % generate_and_save_data allows passing filename as second arg
        generate_and_save_data([], csv_file);
    catch ME
        error('Failed to generate CSV ''%s'': %s', csv_file, ME.message);
    end
end

T = readtable(csv_file);
csvN = height(T);

% allocate
true_state_full = zeros(csvN,4);
meas_full.pos = nan(csvN,2);
meas_full.vel = nan(csvN,2);

% optional sensors
if ismember('accel3_x', T.Properties.VariableNames)
    meas_full.accel3 = nan(csvN,3);
end
if ismember('gyro3_x', T.Properties.VariableNames)
    meas_full.gyro3 = nan(csvN,3);
end
if ismember('mag3_x', T.Properties.VariableNames)
    meas_full.mag3 = nan(csvN,3);
end
if ismember('gps_x', T.Properties.VariableNames)
    meas_full.gps = nan(csvN,2);
end
if ismember('baro', T.Properties.VariableNames)
    meas_full.baro = nan(csvN,1);
end
if ismember('meas_heading_x', T.Properties.VariableNames)
    meas_full.heading = nan(csvN,2);
end

% populate
for ii = 1:csvN
    % true state: try to read x,y and velocity if available
    if ismember('x', T.Properties.VariableNames) && ismember('y', T.Properties.VariableNames)
        % if v and quaternion columns exist, we prefer computing vx,vy from them
        if ismember('v', T.Properties.VariableNames) && ismember('q1', T.Properties.VariableNames)
            v_k = T.v(ii);
            qw = T.q1(ii); qx = T.q2(ii); qy = T.q3(ii); qz = T.q4(ii);
            R = [1-2*(qy^2+qz^2), 2*(qx*qy- qz*qw), 2*(qx*qz+ qy*qw);
                 2*(qx*qy+ qz*qw), 1-2*(qx^2+qz^2), 2*(qy*qz- qx*qw);
                 2*(qx*qz- qy*qw), 2*(qy*qz+ qx*qw), 1-2*(qx^2+qy^2)];
            vel_world = (R * [v_k;0;0]);
            true_state_full(ii,:) = [T.x(ii), T.y(ii), vel_world(1), vel_world(2)];
        elseif ismember('vx', T.Properties.VariableNames) && ismember('vy', T.Properties.VariableNames)
            true_state_full(ii,:) = [T.x(ii), T.y(ii), T.vx(ii), T.vy(ii)];
        else
            true_state_full(ii,:) = [T.x(ii), T.y(ii), 0, 0];
        end
    else
        true_state_full(ii,:) = [0 0 0 0];
    end
    % measurements: prefer meas_pos_x/meas_pos_y (existing) or meas.pos
    if ismember('meas_pos_x', T.Properties.VariableNames) && ismember('meas_pos_y', T.Properties.VariableNames)
        meas_full.pos(ii,:) = [T.meas_pos_x(ii), T.meas_pos_y(ii)];
    elseif ismember('x', T.Properties.VariableNames) && ismember('y', T.Properties.VariableNames)
        meas_full.pos(ii,:) = [T.x(ii), T.y(ii)];
    end
    if ismember('meas_vel_x', T.Properties.VariableNames) && ismember('meas_vel_y', T.Properties.VariableNames)
        meas_full.vel(ii,:) = [T.meas_vel_x(ii), T.meas_vel_y(ii)];
    elseif ismember('v', T.Properties.VariableNames) && ismember('q1', T.Properties.VariableNames)
        v_k = T.v(ii); qw = T.q1(ii); qx = T.q2(ii); qy = T.q3(ii); qz = T.q4(ii);
        R = [1-2*(qy^2+qz^2), 2*(qx*qy- qz*qw), 2*(qx*qz+ qy*qw);
             2*(qx*qy+ qz*qw), 1-2*(qx^2+qz^2), 2*(qy*qz- qx*qw);
             2*(qx*qz- qy*qw), 2*(qy*qz+ qx*qw), 1-2*(qx^2+qy^2)];
        vel_world = (R * [v_k;0;0]);
        meas_full.vel(ii,:) = vel_world(1:2)';
    end
    if isfield(meas_full,'accel3') && ismember('accel3_x', T.Properties.VariableNames), meas_full.accel3(ii,:) = [T.accel3_x(ii), T.accel3_y(ii), T.accel3_z(ii)]; end
    if isfield(meas_full,'gyro3') && ismember('gyro3_x', T.Properties.VariableNames), meas_full.gyro3(ii,:) = [T.gyro3_x(ii), T.gyro3_y(ii), T.gyro3_z(ii)]; end
    if isfield(meas_full,'mag3') && ismember('mag3_x', T.Properties.VariableNames), meas_full.mag3(ii,:) = [T.mag3_x(ii), T.mag3_y(ii), T.mag3_z(ii)]; end
    if isfield(meas_full,'gps') && ismember('gps_x', T.Properties.VariableNames), meas_full.gps(ii,:) = [T.gps_x(ii), T.gps_y(ii)]; end
    if isfield(meas_full,'baro') && ismember('baro', T.Properties.VariableNames), meas_full.baro(ii) = T.baro(ii); end
    if isfield(meas_full,'heading') && ismember('meas_heading_x', T.Properties.VariableNames), meas_full.heading(ii,:) = [T.meas_heading_x(ii), T.meas_heading_y(ii)]; end
end
end
