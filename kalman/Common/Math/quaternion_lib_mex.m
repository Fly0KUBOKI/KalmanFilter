function varargout = quaternion_lib_mex(action, varargin)
    % QUATERNION_LIB_MEX  QuaternionLib のC++実装ラッパー
    % C++のmex_quaternion_libを呼び出す
    
    persistent use_mex;
    
    if isempty(use_mex)
        % MEXファイルの存在確認
        use_mex = exist('mex_quaternion_lib', 'file') == 3;
        if ~use_mex
            warning('mex_quaternion_lib not found, falling back to MATLAB implementation');
        end
    end
    
    if ~use_mex
        % フォールバック: MATLAB実装を使用
        error('quaternion_lib_mex:NoMEX', 'MEX implementation not available');
    end
    
    % C++実装を呼び出し
    [varargout{1:nargout}] = mex_quaternion_lib(action, varargin{:});
end
