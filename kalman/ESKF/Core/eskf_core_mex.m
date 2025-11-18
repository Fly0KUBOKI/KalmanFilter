function varargout = eskf_core_mex(func_name, varargin)
% ESKF_CORE_MEX  ESKF core functions with automatic MEX fallback
%
% This wrapper auto-detects mex_eskf_core and falls back to MATLAB if unavailable.
%
% Usage:
%   [p,v,q,ba,bg] = eskf_core_mex('integrate_nominal', p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_thr, accel_thr)
%   q = eskf_core_mex('update_accel', q, a_meas, scale_factor)
%   [q, P, K, dx] = eskf_core_mex('update_mag', q, P, m_meas, m_world, R_mag)
%   [p, v, P, K, dx] = eskf_core_mex('update_gps', p, v, P, gps_pos, gps_origin, R_gps)
%   [p, P, K, dx] = eskf_core_mex('update_baro', p, P, pressure, gps_origin, R_baro)

persistent use_mex
if isempty(use_mex)
    use_mex = exist('mex_eskf_core', 'file') == 3;
    if use_mex
        fprintf('[eskf_core_mex] Using MEX acceleration\n');
    else
        fprintf('[eskf_core_mex] MEX not found, using MATLAB implementation\n');
        fprintf('  To enable MEX: cd cpp; build_mex;\n');
    end
end

if use_mex
    try
        [varargout{1:nargout}] = mex_eskf_core(func_name, varargin{:});
        return;
    catch ME
        warning('eskf_core_mex:fallback', 'MEX call failed, falling back to MATLAB: %s', ME.message);
        use_mex = false;
    end
end

% MATLAB fallback
switch func_name
    case 'integrate_nominal'
        [varargout{1:nargout}] = integrate_nominal_matlab(varargin{:});
    case 'update_accel'
        varargout{1} = update_accel_matlab(varargin{:});
    case 'update_mag'
        [varargout{1:nargout}] = update_mag_matlab(varargin{:});
    case 'update_gps'
        [varargout{1:nargout}] = update_gps_matlab(varargin{:});
    case 'update_baro'
        [varargout{1:nargout}] = update_baro_matlab(varargin{:});
    case 'predict_covariance'
        varargout{1} = predict_covariance_matlab(varargin{:});
    case 'inject_error_state'
        [varargout{1:nargout}] = inject_error_state_matlab(varargin{:});
    otherwise
        error('Unknown function: %s', func_name);
end
end

% MATLAB implementations (fallback)
function [p, v, q, ba, bg] = integrate_nominal_matlab(p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_noise_threshold, accel_noise_threshold)
    [p, v, q, ba, bg] = integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_noise_threshold, accel_noise_threshold);
end

function q = update_accel_matlab(q, a_meas, scale_factor)
    if nargin < 3, scale_factor = 1.0; end
    
    % 健全性チェック
    a_norm = norm(a_meas);
    if a_norm < 0.1 || abs(a_norm - 9.81) > 3.0
        return;
    end
    
    % 現在のYaw取得
    euler_current = QuaternionLib.to_euler(q);
    yaw_current = euler_current(3);
    
    % Roll/Pitch計算
    roll_measured = atan2d(a_meas(2), a_meas(3));
    pitch_measured = atan2d(-a_meas(1), sqrt(a_meas(2)^2 + a_meas(3)^2));
    
    roll_current = euler_current(1);
    pitch_current = euler_current(2);
    
    % スケーリング適用
    roll_target = roll_current + (roll_measured - roll_current) * scale_factor;
    pitch_target = pitch_current + (pitch_measured - pitch_current) * scale_factor;
    
    % 新しいクォータニオン生成
    q = QuaternionLib.from_euler([roll_target; pitch_target; yaw_current]);
    q = QuaternionLib.normalize(q);
end

function [q, P, K, dx] = update_mag_matlab(q, P, m_meas, m_world, R_mag)
    % 回転行列取得
    Rb = QuaternionLib.to_rotation_matrix(q);
    
    % 予測磁場
    h_mag = Rb' * m_world;
    
    % 観測行列
    H = [zeros(3,6), RotationLib.skew_symmetric(h_mag), zeros(3,6)];
    
    % イノベーション・カルマンゲイン
    [y, S, ~] = kalman_filter_core('compute_innovation_and_S', m_meas, h_mag, H, P, R_mag, struct());
    K = kalman_filter_core('compute_kalman_gain', P, H, S);
    dx = K * y;
    
    % 状態更新
    if numel(dx) >= 9
        dtheta = [0; 0; dx(9)];
        dq = QuaternionLib.small_angle_quat(dtheta);
        q = QuaternionLib.multiply(q, dq);
        q = QuaternionLib.normalize(q);
    end
    
    % 共分散更新
    [~, P] = kalman_filter_core('update_state_covariance', zeros(15,1), P, K, H, y, R_mag);
end

function [p, v, P, K, dx] = update_gps_matlab(p, v, P, gps_pos, gps_origin, R_gps)
    % GPS->ローカル座標変換
    lat0 = gps_origin(1); lon0 = gps_origin(2); alt0 = gps_origin(3);
    lat = gps_pos(1); lon = gps_pos(2); alt = gps_pos(3);
    
    y_m = (lat - lat0) / 9.0e-6;
    x_m = (lon - lon0) / (9.0e-6 / cosd(lat0));
    z_m = alt - alt0;
    p_local = [x_m; y_m; z_m];
    
    % 観測行列（位置のみ）
    H = [eye(3), zeros(3,12)];
    
    % イノベーション・カルマンゲイン
    [y, S, ~] = kalman_filter_core('compute_innovation_and_S', p_local, p, H, P, R_gps, struct());
    K = kalman_filter_core('compute_kalman_gain', P, H, S);
    dx = K * y;
    
    % 状態更新
    if numel(dx) >= 6
        p = p + dx(1:3);
        v = v + dx(4:6);
    end
    
    % 共分散更新
    x_dummy = zeros(15,1);
    [~, P] = kalman_filter_core('update_state_covariance', x_dummy, P, K, H, y, R_gps);
end

function [p, P, K, dx] = update_baro_matlab(p, P, pressure, gps_origin, R_baro)
    % 気圧->高度変換
    P0 = 101325;
    alt_meas = 44330 * (1 - (pressure / P0)^0.1903);
    alt_origin = gps_origin(3);
    z_local = alt_meas - alt_origin;
    
    % 観測行列（Z軸のみ）
    H = zeros(1,15);
    H(3) = 1;
    
    % イノベーション・カルマンゲイン
    [y, S, ~] = kalman_filter_core('compute_innovation_and_S', z_local, p(3), H, P, R_baro, struct());
    K = kalman_filter_core('compute_kalman_gain', P, H, S);
    dx = K * y;
    
    % 状態更新
    if numel(dx) >= 3
        p(3) = p(3) + dx(3);
    end
    
    % 共分散更新
    x_dummy = zeros(15,1);
    [~, P] = kalman_filter_core('update_state_covariance', x_dummy, P, K, H, y, R_baro);
end

function P_new = predict_covariance_matlab(P, q, a_meas, ba, w_meas, bg, Q, dt)
    P_new = ESKFCovariancePrediction.predict(P, q, a_meas, ba, w_meas, bg, Q, dt);
end

function [p, v, q, ba, bg] = inject_error_state_matlab(p, v, q, ba, bg, dx)
    nominal = struct('p', p, 'v', v, 'q', q, 'ba', ba, 'bg', bg);
    nominal = ESKFErrorInjection.inject_error_state(nominal, dx);
    p = nominal.p;
    v = nominal.v;
    q = nominal.q;
    ba = nominal.ba;
    bg = nominal.bg;
end
