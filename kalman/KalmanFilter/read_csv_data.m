function [sensor_data, truth_data] = read_csv_data(csv_file)
% READ_CSV_DATA - CSVファイルからセンサデータと真値を読み取る
%
% Inputs:
%   csv_file: CSVファイルのパス
%
% Outputs:
%   sensor_data: センサデータ構造体
%   truth_data: 真値データ構造体

if ~exist(csv_file, 'file')
    error('CSVファイルが見つかりません: %s', csv_file);
end

% CSVデータを読み取り
data = readtable(csv_file);

% 時間データからdtを計算
t = data.t;
dt = diff(t);
dt = [dt(1); dt]; % 最初の要素は前の値と同じに設定

% センサデータ構造体を作成
sensor_data = struct();
sensor_data.t = single(t);
sensor_data.dt = single(dt);

% 6軸センサ (400Hz相当)
sensor_data.accel3 = single([data.accel3_x, data.accel3_y, data.accel3_z]);
sensor_data.gyro3 = single([data.gyro3_x, data.gyro3_y, data.gyro3_z]);

% 地磁気センサ (100Hz相当)
sensor_data.mag3 = single([data.mag3_x, data.mag3_y, data.mag3_z]);

% 気圧センサ (50Hz相当)
sensor_data.baro = single(data.baro);

% GPS (10Hz相当)
sensor_data.gps = single([data.gps_x, data.gps_y]);

% 測定値（ノイズ付き）
sensor_data.meas_pos = single([data.meas_pos_x, data.meas_pos_y]);
sensor_data.meas_vel = single([data.meas_vel_x, data.meas_vel_y]);
sensor_data.meas_heading = single([data.meas_heading_x, data.meas_heading_y]);

% 真値データ構造体を作成
truth_data = struct();
truth_data.t = single(t);
truth_data.pos = single([data.x, data.y, data.z]);
truth_data.vel = single([data.vx, data.vy, data.vz]);
truth_data.accel = single([data.ax, data.ay, zeros(length(t), 1)]);
truth_data.theta = single(data.theta);
truth_data.omega = single(data.omega);

fprintf('CSVデータを読み込みました: %d 行\n', length(t));

end