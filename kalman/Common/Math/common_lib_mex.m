function varargout = common_lib_mex(func_name, varargin)
    % COMMON_LIB_MEX MEX化されたCommonライブラリの統一インターフェース
    %
    % MEX高速化対応: mex_common_libが利用可能な場合は自動的に使用
    %
    % MathUtils関数:
    %   angle_wrapped = common_lib_mex('wrap_to_pi', angle)
    %   v_normalized = common_lib_mex('normalize_vector', v)
    %   S = common_lib_mex('skew_symmetric', v)
    %   M_sym = common_lib_mex('enforce_symmetry', M)
    %
    % QuaternionLib関数:
    %   q_out = common_lib_mex('quat_multiply', q1, q2)
    %   q_norm = common_lib_mex('quat_normalize', q)
    %   R = common_lib_mex('quat_to_rotm', q)
    %   q = common_lib_mex('quat_from_euler', roll, pitch, yaw)
    %   [roll, pitch, yaw] = common_lib_mex('quat_to_euler', q)
    %
    % StateValidator関数:
    %   [v_clipped, clipped] = common_lib_mex('clip_velocity', v, max_vel)
    %   [P_valid, valid] = common_lib_mex('validate_covariance', P)
    %
    % SensorFilter関数:
    %   filtered = common_lib_mex('lowpass_filter', signal, prev_filtered, alpha)
    %   is_outlier = common_lib_mex('detect_outlier', measurement, expected, threshold)
    
    persistent use_mex;
    
    % 初回呼び出し時にMEXファイルの存在をチェック
    if isempty(use_mex)
        use_mex = exist('mex_common_lib', 'file') == 3;
        if use_mex
            fprintf('[common_lib_mex] Using MEX acceleration\n');
        else
            fprintf('[common_lib_mex] MEX not found, using MATLAB implementation\n');
            fprintf('  To enable MEX: cd cpp; build_mex;\n');
        end
    end
    
    % MEX実装を使用
    if use_mex
        try
            [varargout{1:nargout}] = mex_common_lib(func_name, varargin{:});
            return;
        catch ME
            warning('common_lib_mex:mexFallback', 'MEX call failed: %s', ME.message);
            use_mex = false;
        end
    end
    
    % MATLABフォールバック実装
    switch func_name
        % MathUtils
        case 'wrap_to_pi'
            varargout{1} = wrap_to_pi_matlab(varargin{:});
        case 'normalize_vector'
            varargout{1} = normalize_vector_matlab(varargin{:});
        case 'skew_symmetric'
            varargout{1} = skew_symmetric_matlab(varargin{:});
        case 'enforce_symmetry'
            varargout{1} = enforce_symmetry_matlab(varargin{:});
            
        % QuaternionLib
        case 'quat_multiply'
            varargout{1} = QuaternionLib.multiply(varargin{:});
        case 'quat_normalize'
            varargout{1} = QuaternionLib.normalize(varargin{:});
        case 'quat_to_rotm'
            varargout{1} = QuaternionLib.to_rotation_matrix(varargin{:});
        case 'quat_from_euler'
            varargout{1} = QuaternionLib.from_euler(varargin{:});
        case 'quat_to_euler'
            [varargout{1:nargout}] = QuaternionLib.to_euler(varargin{:});
            
        % StateValidator
        case 'clip_velocity'
            [varargout{1:nargout}] = StateValidator.clip_velocity(varargin{:});
        case 'validate_covariance'
            [varargout{1:nargout}] = CovarianceRegularizer.regularize(varargin{:});
            
        % SensorFilter
        case 'lowpass_filter'
            varargout{1} = lowpass_filter_matlab(varargin{:});
        case 'detect_outlier'
            varargout{1} = detect_outlier_matlab(varargin{:});
            
        otherwise
            error('common_lib_mex:unknownFunction', 'Unknown function: %s', func_name);
    end
end

%% MATLAB Fallback Implementations

function angle_wrapped = wrap_to_pi_matlab(angle)
    angle_wrapped = mod(angle + pi, 2*pi) - pi;
end

function v_normalized = normalize_vector_matlab(v)
    v = v(:);
    n = norm(v);
    if n < 1e-9
        v_normalized = zeros(size(v));
    else
        v_normalized = v / n;
    end
end

function S = skew_symmetric_matlab(v)
    v = v(:);
    S = [0, -v(3), v(2);
         v(3), 0, -v(1);
         -v(2), v(1), 0];
end

function M_sym = enforce_symmetry_matlab(M)
    M_sym = (M + M') / 2;
end

function filtered = lowpass_filter_matlab(signal, prev_filtered, alpha)
    filtered = alpha * signal + (1 - alpha) * prev_filtered;
end

function is_outlier = detect_outlier_matlab(measurement, expected, threshold)
    diff = measurement - expected;
    is_outlier = norm(diff) > threshold;
end
