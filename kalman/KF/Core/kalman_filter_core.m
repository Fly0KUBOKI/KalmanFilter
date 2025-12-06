function varargout = kalman_filter_core(func_name, varargin)
    % KALMAN_FILTER_CORE  カルマンフィルタの共通関数をまとめたコアファイル
    % MATLAB implementation (MEX disabled for stability)
    
    % MEXを無効化してMATLABを使用
    persistent use_mex;
    if isempty(use_mex)
        use_mex = false;  % MEXを強制的に無効化
        fprintf('[kalman_filter_core] Using MATLAB implementation (MEX disabled)\n');
    end
    
    % MATLAB fallback implementation
    switch func_name
        case 'compute_innovation_and_S'
            [varargout{1:nargout}] = compute_innovation_and_S_matlab(varargin{:});
        case 'compute_kalman_gain'
            varargout{1} = compute_kalman_gain_matlab(varargin{:});
        case 'update_state_covariance'
            [varargout{1:nargout}] = update_state_covariance_matlab(varargin{:});
        otherwise
            error('kalman_filter_core:unknown', 'Unknown function: %s', func_name);
    end
end

function [y, S, R_out] = compute_innovation_and_S_matlab(z, h, H, P, R, ~)
    % Innovation
    y = z - h;
    % Innovation covariance
    S = H * P * H' + R;
    R_out = R;
end

function K = compute_kalman_gain_matlab(P, H, S)
    % Kalman gain
    K = (P * H') / S;
end

function [x_upd, P_upd] = update_state_covariance_matlab(x, P, K, H, y, R)
    % State update
    dx = K * y;
    x_upd = x + dx;
    
    % Joseph form covariance update for numerical stability
    I_KH = eye(size(P)) - K * H;
    P_upd = I_KH * P * I_KH' + K * R * K';
    
    % Ensure symmetry
    P_upd = (P_upd + P_upd') / 2;
end
