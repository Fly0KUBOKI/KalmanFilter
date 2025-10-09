function [meas, state_curr, is_end] = csv_reader(params, k)
% csv_reader.m
% CSVファイルからステップkのデータを読み取り、測定値と真値状態を返す
% 
% Inputs:
%   params - パラメータ構造体（params.data.fileにCSVファイルパス）
%   k      - 読み取るステップ（行番号）
%
% Outputs:
%   meas       - 測定値構造体
%   state_curr - 真値状態ベクトル
%   is_end     - CSVファイルの終端に達した場合はtrue

persistent T csvN file_path file_datenum

% Validate params
if ~(isfield(params,'data') && isfield(params.data,'source') && strcmpi(params.data.source,'csv') && isfield(params.data,'file'))
    error('CSV data source required. Set params.data.source=''csv'' and params.data.file.');
end

% Resolve full path
requested_path = params.data.file;
% If the requested path is relative, make it absolute relative to pwd
if ~ischar(requested_path)
    error('params.data.file must be a file path string');
end
if ~isfolder(fileparts(requested_path)) && ~isfile(requested_path)
    % leave as-is; readtable will error if not found
end

% Check file modification time and reload when changed or when first call
file_info = dir(requested_path);
need_reload = false;
if isempty(file_info)
    error('CSV file not found: %s', requested_path);
end
if isempty(T)
    need_reload = true;
elseif isempty(file_path) || ~strcmp(file_path, requested_path)
    need_reload = true;
else
    % datenum changed -> reload
    if file_info.datenum ~= file_datenum
        need_reload = true;
    end
end

if need_reload
    T = readtable(requested_path);
    csvN = height(T);
    file_path = requested_path;
    file_datenum = file_info.datenum;
    fprintf('CSV file loaded/reloaded: %s (%d rows)\n', requested_path, csvN);
end

% CSVファイルの終端チェック
is_end = (k > csvN);
if is_end
    meas = struct();
    state_curr = [];
    return;
end

% 真値状態の読み取り（位置と速度）
state_curr = [T.x(k); T.y(k); T.vx(k); T.vy(k)];

% 測定値の読み取り
meas = struct();

% 基本測定値（位置、速度）
meas.pos = [T.meas_pos_x(k), T.meas_pos_y(k)];
meas.vel = [T.meas_vel_x(k), T.meas_vel_y(k)];

% オプション測定値（存在する場合のみ読み取り）
if ismember('accel3_x', T.Properties.VariableNames)
    meas.accel3 = [T.accel3_x(k), T.accel3_y(k), T.accel3_z(k)];
end
if ismember('gyro3_x', T.Properties.VariableNames)
    meas.gyro3 = [T.gyro3_x(k), T.gyro3_y(k), T.gyro3_z(k)];
end
if ismember('mag3_x', T.Properties.VariableNames)
    meas.mag3 = [T.mag3_x(k), T.mag3_y(k), T.mag3_z(k)];
end
if ismember('gps_x', T.Properties.VariableNames)
    meas.gps = [T.gps_x(k), T.gps_y(k)];
end
if ismember('baro', T.Properties.VariableNames)
    meas.baro = T.baro(k);
end
if ismember('meas_heading_x', T.Properties.VariableNames)
    meas.heading = [T.meas_heading_x(k), T.meas_heading_y(k)];
end

end