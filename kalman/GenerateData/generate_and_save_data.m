function fname = generate_and_save_data(params, fname)
% generate_and_save_data - sim_generate を使って模擬データを作り CSV に保存する
% Usage:
%  fname = generate_and_save_data(params)
%  fname = generate_and_save_data(params, 'data.csv')

if nargin<1 || isempty(params)
    params = config_params();
end

if nargin<2 || isempty(fname)
    fname = fullfile(pwd,'sim_data.csv');
end

[t, state, meas, params] = sim_generate(params);

% 結果をテーブルにして保存
N = numel(t);
T = table();
T.t = t;
% state columns (if present)
if size(state,2) >= 1, T.x = state(:,1); end
if size(state,2) >= 2, T.y = state(:,2); end
if size(state,2) >= 3, T.vx = state(:,3); end
if size(state,2) >= 4, T.vy = state(:,4); end
if size(state,2) >= 5, T.theta = state(:,5); end
if size(state,2) >= 6, T.ax = state(:,6); end
if size(state,2) >= 7, T.ay = state(:,7); end
if size(state,2) >= 8, T.omega = state(:,8); end
if size(state,2) >= 9, T.z = state(:,9); end
if size(state,2) >= 10, T.vz = state(:,10); end

% measurements
if isfield(meas,'pos'), T.meas_pos_x = meas.pos(:,1); T.meas_pos_y = meas.pos(:,2); end
if isfield(meas,'vel'), T.meas_vel_x = meas.vel(:,1); T.meas_vel_y = meas.vel(:,2); end
if isfield(meas,'accel') && isvector(meas.accel), T.meas_accel = meas.accel; end
if isfield(meas,'heading'), T.meas_heading_x = meas.heading(:,1); T.meas_heading_y = meas.heading(:,2); end
if isfield(meas,'accel3'), T.accel3_x = meas.accel3(:,1); T.accel3_y = meas.accel3(:,2); T.accel3_z = meas.accel3(:,3); end
if isfield(meas,'gyro3'), T.gyro3_x = meas.gyro3(:,1); T.gyro3_y = meas.gyro3(:,2); T.gyro3_z = meas.gyro3(:,3); end
if isfield(meas,'mag3'), T.mag3_x = meas.mag3(:,1); T.mag3_y = meas.mag3(:,2); T.mag3_z = meas.mag3(:,3); end
if isfield(meas,'gps'), T.gps_x = meas.gps(:,1); T.gps_y = meas.gps(:,2); end
if isfield(meas,'baro'), T.baro = meas.baro; end

writetable(T, fname);
fprintf('Saved simulated data to %s (N=%d)\n', fname, N);

end
