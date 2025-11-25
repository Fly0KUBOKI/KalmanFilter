function varargout = kalman_filter_core(func_name, varargin)
    % KALMAN_FILTER_CORE  カルマンフィルタの共通関数をまとめたコアファイル
    
    % 強制的にMATLAB実装を使用
    use_mex = false;
    
    if use_mex
        try
            [varargout{1:nargout}] = mex_kalman_filter_core(func_name, varargin{:});
            return;
        catch ME
            warning('kalman_filter_core:mexFallback', 'MEX call failed, falling back to MATLAB: %s', ME.message);
        end
    end
    
    % MATLAB実装
    switch func_name
        case 'compute_innovation_and_S'
            [varargout{1:nargout}] = compute_innovation_and_S_matlab(varargin{:});
        case 'compute_kalman_gain'
            [varargout{1:nargout}] = compute_kalman_gain_matlab(varargin{:});
        case 'update_state_covariance'
            [varargout{1:nargout}] = update_state_covariance_matlab(varargin{:});
        otherwise
            error('Unknown function or not implemented in MATLAB fallback: %s', func_name);
    end
end

function [y, S, R_out] = compute_innovation_and_S_matlab(z, h, H, P_pred, R, params)
    y = z - h;
    S = H * P_pred * H' + R;
    S = (S + S') / 2;
    R_out = R;
end

function K = compute_kalman_gain_matlab(P, H, S)
    % K = P * H' / S
    try
        U = chol(S);
        tmp = P * H';
        K = (U \ (U' \ tmp'))';
    catch
        K = P * H' / S;
    end
end

function [x_upd, P_upd] = update_state_covariance_matlab(x_pred, P_pred, K, H, y, R)
    % x_upd = x_pred + K * y
    x_upd = x_pred + K * y;
    
    % P_upd = (I - K*H) * P_pred * (I - K*H)' + K*R*K' (Joseph form)
    I = eye(size(P_pred));
    I_KH = I - K * H;
    P_upd = I_KH * P_pred * I_KH' + K * R * K';
    P_upd = (P_upd + P_upd') / 2;
end
