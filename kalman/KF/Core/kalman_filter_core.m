function varargout = kalman_filter_core(func_name, varargin)
    % KALMAN_FILTER_CORE  カルマンフィルタの共通関数をまとめたコアファイル
    % MEX専用実装 - MATLAB fallbackは削除済み
    
    % MEX実装の自動検出
    persistent use_mex;
    if isempty(use_mex)
        use_mex = exist('mex_kalman_filter_core', 'file') == 3;
        if ~use_mex
            error('kalman_filter_core:noMEX', 'MEX implementation not found. Please build mex_kalman_filter_core.');
        end
        fprintf('kalman_filter_core: MEX implementation detected and enabled\n');
    end
    
    [varargout{1:nargout}] = mex_kalman_filter_core(func_name, varargin{:});
end
