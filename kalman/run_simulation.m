function run_simulation()
% RUN_SIMULATION  メイン実行スクリプト（簡易ESKFデモ）
%
% このスクリプトは、GenerateData/observation.csv を読み込み、簡易的な
% ESKF フィルタを実行して Results/estimation.csv を出力し、グラフを表示します.

projRoot = fileparts(mfilename('fullpath'));

% MATLAB パスにサブフォルダを追加
addpath(fullfile(projRoot, 'KalmanFilter'));
addpath(fullfile(projRoot, 'GenerateData'));
addpath(fullfile(projRoot, 'Graph'));
addpath(fullfile(projRoot, 'KalmanFilter', 'Common Calculations'));

dataDir = fullfile(projRoot, 'GenerateData');
obsFile = fullfile(dataDir, 'sensor_data.csv');
if ~exist(obsFile, 'file')
    error('sensor_data.csv が見つかりません: %s', obsFile);
end

obs = read_csv(obsFile);

% 推定ノイズパラメータ
[Q, R_gps, R_mag, R_baro] = estimate_noise_params(obs);

% 設定
if length(obs.time) < 2
    error('観測データが短すぎます');
end
settings.dt = mean(diff(obs.time));
settings.freq_mag = 4;   % 1/4ステップ
settings.freq_baro = 8;  % 1/8ステップ
settings.freq_gps = 40;  % 1/40ステップ
settings.Q = Q; settings.R_gps = R_gps; settings.R_mag = R_mag; settings.R_baro = R_baro;

% --- 簡易 ESKF 実行ループ（最小実装） ---
% 初期化
N = numel(obs.time);
results.time = obs.time(:)';
results.p = zeros(3, N);
results.v = zeros(3, N);
results.euler = zeros(3, N);

% 初期ノミナル状態
% p,v,q,ba,bg
p = zeros(3,1);
v = zeros(3,1);
q = eskf_utils('quatnormalize', [1;0;0;0]);
ba = zeros(3,1);
bg = zeros(3,1);

% 誤差共分散 P (15x15)
P = eye(15) * 0.01;

dt = settings.dt;
g = [0;0;-9.81];

for k=1:N
    % センサ計測
    a = [obs.ax(k); obs.ay(k); obs.az(k)];
    w = [obs.wx(k); obs.wy(k); obs.wz(k)];

    % バイアス補正
    a_corr = a - ba;
    w_corr = w - bg;

    % ノミナル状態の数値積分（first-order）
    % update quaternion
    delta_q = eskf_utils('small_angle_quat', w_corr * dt);
    q = eskf_utils('quatmultiply', q, delta_q);
    q = eskf_utils('quatnormalize', q);
    Rb = eskf_utils('quat_to_rotm', q);
    a_world = Rb * a_corr + g;
    v = v + a_world * dt;
    p = p + v * dt;

    % 簡易予測の共分散更新（対角のみ、Qは settings.Q）
    P = P + settings.Q * dt;

    % 更新：加速度（静止判定で roll/pitch 補正）
    a_norm = norm(a);
    is_stationary = abs(a_norm - 9.81) < 0.5;
    if is_stationary
        g_world = [0;0;-9.81];
        h_accel = Rb' * g_world;
        y = (a - ba) - h_accel;
        H = [zeros(3,3), zeros(3,3), -eskf_utils('skew', h_accel), -eye(3), zeros(3,3)];
        S = H * P * H' + eye(3) * 0.01;
        K = P * H' / S;
        dx = K * y;
        % apply small corrections (roll/pitch only)
        dtheta_rp = [dx(7); dx(8); 0];
        dq = eskf_utils('small_angle_quat', dtheta_rp);
        q = eskf_utils('quatmultiply', q, dq);
        q = eskf_utils('quatnormalize', q);
        ba = ba + dx(10:12);
        P = (eye(15) - K*H) * P;
    end

    % ここでは GPS/mag/baro の周期的更新は簡略化して無視（後で追加可）

    % 保存
    results.p(:,k) = p;
    results.v(:,k) = v;
    % convert quat to euler (ZYX yaw-pitch-roll -> [roll;pitch;yaw])
    Rm = eskf_utils('quat_to_rotm', q);
    yaw = atan2(Rm(2,1), Rm(1,1));
    pitch = asin(-Rm(3,1));
    roll = atan2(Rm(3,2), Rm(3,3));
    results.euler(:,k) = [roll; pitch; yaw];
end

outDir = fullfile(projRoot, 'Results');
if ~exist(outDir,'dir'), mkdir(outDir); end
outFile = fullfile(outDir, 'estimation.csv');

% 保存
T = table(results.time(:), results.p(1,:)', results.p(2,:)', results.p(3,:)', ...
    results.v(1,:)', results.v(2,:)', results.v(3,:)', ...
    results.euler(1,:)', results.euler(2,:)', results.euler(3,:)');
T.Properties.VariableNames = {'time','px','py','pz','vx','vy','vz','roll','pitch','yaw'};
writetable(T, outFile);

% 可視化
plot_csv(outFile);

fprintf('Estimation saved to %s\n', outFile);
end
