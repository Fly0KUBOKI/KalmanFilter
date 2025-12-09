% C++実装の簡易テスト
fprintf('=== C++実装テスト ===\n');

% パス追加
addpath('cpp/bin');
addpath('ESKF');

% MEXが利用可能か確認
if exist('mex_meukf_step_v2', 'file') ~= 3
    error('mex_meukf_step_v2が見つかりません');
end
fprintf('MEX確認: OK\n');

% 簡易状態作成
state.p = zeros(3,1);
state.v = zeros(3,1);
state.q = [1;0;0;0];
state.ba = zeros(3,1);
state.bg = zeros(3,1);
state.P = eye(15) * 0.01;

% センサーデータ
sensor.accel = [0.1; 0.1; 9.81];
sensor.gyro = [0.01; 0.01; 0.01];
sensor.mag = [0.3; 0; 0.5];
sensor.gps_pos = zeros(3,1);
sensor.alt_baro = 0;
sensor.dt = 0.01;
sensor.update_accel = true;
sensor.update_gyro = false;
sensor.update_mag = false;
sensor.update_gps = false;
sensor.update_baro = false;

% パラメータ
params.g = [0; 0; -9.81];
params.mag_ref = [0; 50; 0];
params.noise_accel = [0.1; 0.1; 0.1];
params.noise_gyro = [0.01; 0.01; 0.01];
params.noise_ba = [0.001; 0.001; 0.001];
params.noise_bg = [0.0001; 0.0001; 0.0001];
params.noise_mag = [0.1; 0.1; 0.1];
params.noise_gps = [1; 1; 1];
params.noise_baro = 0.5;
params.alpha = 1e-3;
params.beta = 2;
params.kappa = 0;

% MEX呼び出し
try
    fprintf('MEX呼び出し中...\n');
    [new_state, status, debug_info] = mex_meukf_step_v2(state, sensor, params);
    fprintf('MEX呼び出し成功\n');
    fprintf('Status: %d\n', status);
    fprintf('Position: [%.3f, %.3f, %.3f]\n', new_state.p);
    fprintf('Quaternion: [%.3f, %.3f, %.3f, %.3f]\n', new_state.q);
catch ME
    fprintf('MEXエラー: %s\n', ME.message);
    for k = 1:min(3, length(ME.stack))
        fprintf('  %s (line %d)\n', ME.stack(k).name, ME.stack(k).line);
    end
end

fprintf('\nテスト完了\n');
