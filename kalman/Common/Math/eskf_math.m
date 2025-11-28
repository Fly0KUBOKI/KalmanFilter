function varargout = eskf_math(func_name, varargin)
    % ESKF_MATH  Wrapper for C++ stateless ESKF computation library
    %
    % This is a pure computation library (like math.h functions)
    % No state management - all state is managed by MATLAB
    %
    % Usage examples:
    %   q_new = eskf_math('quaternion_integration', q, w, dt);
    %   [p_new, v_new] = eskf_math('pv_integration', p, v, a_world, g, dt, prev_a, prev_v, use_ab2, max_accel, max_vel);
    %   F = eskf_math('compute_F_matrix', q, a_meas, ba, w_meas, bg, dt);
    %   P_new = eskf_math('covariance_prediction', P, F, Q);
    %   [p, v, q, ba, bg] = eskf_math('inject_error_state', p, v, q, ba, bg, dx);
    %   [x_new, P_new, K, S] = eskf_math('kalman_update', x, P, y, H, R);
    %   q_rp = eskf_math('accel_to_quaternion', a_meas, scale);
    %   m_body = eskf_math('mag_observation_prediction', q, m_world);
    %   pos_local = eskf_math('gps_to_local', gps_pos, origin_pos);
    %   altitude = eskf_math('pressure_to_altitude', pressure);
    
    % Check if MEX is available
    persistent use_mex;
    if isempty(use_mex)
        use_mex = exist('mex_eskf_math', 'file') == 3;
        if use_mex
            fprintf('[eskf_math] Using C++ MEX acceleration\n');
        else
            warning('eskf_math:noMEX', 'mex_eskf_math not found, falling back to MATLAB');
        end
    end
    
    % Call MEX if available
    if use_mex
        try
            [varargout{1:nargout}] = mex_eskf_math(func_name, varargin{:});
            return;
        catch ME
            warning('eskf_math:mexFailed', 'MEX call failed: %s. Falling back to MATLAB', ME.message);
            use_mex = false;  % Disable MEX for future calls
        end
    end
    
    % MATLAB fallback implementation
    switch func_name
        case 'quaternion_integration'
            [varargout{1:nargout}] = quaternion_integration_matlab(varargin{:});
        case 'accel_to_quaternion'
            [varargout{1:nargout}] = accel_to_quaternion_matlab(varargin{:});
        case 'pv_integration'
            [varargout{1:nargout}] = pv_integration_matlab(varargin{:});
        case 'compute_F_matrix'
            [varargout{1:nargout}] = compute_F_matrix_matlab(varargin{:});
        case 'covariance_prediction'
            [varargout{1:nargout}] = covariance_prediction_matlab(varargin{:});
        case 'inject_error_state'
            [varargout{1:nargout}] = inject_error_state_matlab(varargin{:});
        case 'kalman_update'
            [varargout{1:nargout}] = kalman_update_matlab(varargin{:});
        case 'mag_observation_prediction'
            [varargout{1:nargout}] = mag_observation_matlab(varargin{:});
        case 'gps_to_local'
            [varargout{1:nargout}] = gps_to_local_matlab(varargin{:});
        case 'pressure_to_altitude'
            [varargout{1:nargout}] = pressure_to_altitude_matlab(varargin{:});
        otherwise
            error('Unknown function: %s', func_name);
    end
end

%% MATLAB Implementations

function q_new = quaternion_integration_matlab(q, w, dt)
    % Quaternion integration from angular velocity
    w_dt = w * dt;
    w_dt_norm = norm(w_dt);
    
    if w_dt_norm > 1e-15
        half_angle = w_dt_norm / 2;
        if half_angle > 1e-6
            sin_half = sin(half_angle);
            cos_half = cos(half_angle);
            w_unit = w_dt / w_dt_norm;
            delta_q = [cos_half; w_unit * sin_half];
        else
            % Taylor expansion
            w_norm_sq = w_dt_norm^2;
            delta_q = [1 - w_norm_sq/8; w_dt/2 * (1 - w_norm_sq/24)];
        end
        q_new = QuaternionLib.multiply(q, delta_q);
        q_new = QuaternionLib.normalize(q_new);
    else
        q_new = q;
    end
end

function q_rp = accel_to_quaternion_matlab(a_meas, scale_factor)
    % Convert acceleration to roll/pitch quaternion
    a_norm = norm(a_meas);
    if a_norm < 1e-6
        q_rp = [1; 0; 0; 0];
        return;
    end
    
    a_unit = a_meas / a_norm;
    roll = atan2(a_unit(2), a_unit(3)) * scale_factor;
    pitch = atan2(-a_unit(1), sqrt(a_unit(2)^2 + a_unit(3)^2)) * scale_factor;
    
    cr = cos(roll/2);
    sr = sin(roll/2);
    cp = cos(pitch/2);
    sp = sin(pitch/2);
    
    q_rp = [cr*cp; sr*cp; cr*sp; 0];
    q_rp = QuaternionLib.normalize(q_rp);
end

function [p_new, v_new] = pv_integration_matlab(p, v, a_world, g, dt, prev_a, prev_v, use_ab2, max_accel, max_vel)
    % Position and velocity integration
    if use_ab2 > 0.5
        % Adams-Bashforth 2
        accel_current = a_world + g;
        accel_prev = prev_a + g;
        dv = (1.5 * accel_current - 0.5 * accel_prev) * dt;
    else
        % Forward Euler
        dv = (a_world + g) * dt;
    end
    
    % Saturation
    dv_norm = norm(dv);
    max_dv = max_accel * dt;
    if dv_norm > max_dv
        dv = dv * (max_dv / dv_norm);
    end
    
    v_new = v + dv;
    
    % Velocity clipping
    v_norm = norm(v_new);
    if v_norm > max_vel
        v_new = v_new * (max_vel / v_norm);
    end
    
    if use_ab2 > 0.5
        dp = (1.5 * v_new - 0.5 * prev_v) * dt;
    else
        dp = v_new * dt;
    end
    
    p_new = p + dp;
end

function F = compute_F_matrix_matlab(q, a_meas, ba, w_meas, bg, dt)
    % State transition matrix for ESKF
    a = a_meas - ba;
    w = w_meas - bg;
    
    R = QuaternionLib.quat2rotm(q);
    a_skew = [0, -a(3), a(2); a(3), 0, -a(1); -a(2), a(1), 0];
    w_skew = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
    
    F = eye(15);
    F(1:3, 4:6) = eye(3) * dt;
    F(4:6, 7:9) = -R * a_skew * dt;
    F(4:6, 10:12) = -R * dt;
    F(7:9, 7:9) = eye(3) - w_skew * dt;
    F(7:9, 13:15) = -eye(3) * dt;
end

function P_new = covariance_prediction_matlab(P, F, Q)
    % Covariance prediction
    P_new = F * P * F' + Q;
    P_new = (P_new + P_new') / 2;
end

function [p_new, v_new, q_new, ba_new, bg_new] = inject_error_state_matlab(p, v, q, ba, bg, dx)
    % Error state injection
    dp = dx(1:3);
    dv = dx(4:6);
    dtheta = dx(7:9);
    dba = dx(10:12);
    dbg = dx(13:15);
    
    p_new = p + dp;
    v_new = v + dv;
    
    theta_norm = norm(dtheta);
    if theta_norm > 1e-8
        half_angle = theta_norm / 2;
        delta_q = [cos(half_angle); sin(half_angle) * dtheta / theta_norm];
    else
        delta_q = [1; dtheta/2];
    end
    
    q_new = QuaternionLib.multiply(q, delta_q);
    q_new = QuaternionLib.normalize(q_new);
    
    ba_new = ba + dba;
    bg_new = bg + dbg;
end

function [x_new, P_new, K, S] = kalman_update_matlab(x, P, y, H, R)
    % Generic Kalman update
    S = H * P * H' + R;
    S = (S + S') / 2;
    K = P * H' / S;
    x_new = x + K * y;
    I_KH = eye(size(P)) - K * H;
    P_new = I_KH * P * I_KH' + K * R * K';
    P_new = (P_new + P_new') / 2;
end

function m_body = mag_observation_matlab(q, m_world)
    % Magnetic field observation prediction
    R = QuaternionLib.quat2rotm(q);
    m_body = R' * m_world;
end

function pos_local = gps_to_local_matlab(gps_pos, origin_pos)
    % GPS to local coordinates (flat earth)
    lat_diff = (gps_pos(1) - origin_pos(1)) * pi / 180;
    lon_diff = (gps_pos(2) - origin_pos(2)) * pi / 180;
    alt_diff = gps_pos(3) - origin_pos(3);
    
    R_earth = 6371000;
    north = lat_diff * R_earth;
    east = lon_diff * R_earth * cos(origin_pos(1) * pi / 180);
    down = -alt_diff;
    
    pos_local = [north; east; down];
end

function altitude = pressure_to_altitude_matlab(pressure)
    % Pressure to altitude (standard atmosphere)
    p0 = 101325;
    T0 = 288.15;
    L = 0.0065;
    g = 9.80665;
    M = 0.0289644;
    R = 8.31447;
    
    exponent = (R * L) / (g * M);
    altitude = (T0 / L) * (1 - (pressure / p0)^exponent);
end
