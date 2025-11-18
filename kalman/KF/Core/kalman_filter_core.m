function varargout = kalman_filter_core(func_name, varargin)
    % KALMAN_FILTER_CORE  カルマンフィルタの共通関数をまとめたコアファイル
    %
    % MEX高速化対応: mex_kalman_filter_coreが利用可能な場合は自動的に使用
    %
    % 使用方法:
    %   P = kalman_filter_core('predict_step', P, q, a_meas, ba, w_meas, bg, Q, dt)
    %   K = kalman_filter_core('compute_kalman_gain', P_pred, H, S)
    %   [x_upd, P_upd] = kalman_filter_core('update_state_covariance', x_pred, P_pred, K, H, y, R)
    %   [y, S, R_out] = kalman_filter_core('compute_innovation_and_S', z, h, H, P_pred, R, params)
    %   F = kalman_filter_core('compute_jacobian', q, a_meas, ba, dt)
    
    persistent use_mex;
    
    % 初回呼び出し時にMEXファイルの存在をチェック
    if isempty(use_mex)
        use_mex = exist('mex_kalman_filter_core', 'file') == 3;
        if use_mex
            fprintf('[kalman_filter_core] MEX acceleration enabled\n');
        else
            fprintf('[kalman_filter_core] Using MATLAB implementation\n');
        end
    end
    
    % MEX実装を使用（compute_innovation_and_S以外）
    % compute_innovation_and_Sはparamsパラメータの処理があるためMATLAB実装を使用
    if use_mex && ~strcmp(func_name, 'compute_innovation_and_S')
        try
            [varargout{1:nargout}] = mex_kalman_filter_core(func_name, varargin{:});
            return;
        catch ME
            warning('kalman_filter_core:mexFallback', 'MEX call failed, falling back to MATLAB: %s', ME.message);
            use_mex = false; % 次回からMATLAB実装を使用
        end
    end
    
    % MATLAB実装を無効化しました。
    % このファイルはMEX実装（mex_kalman_filter_core）を必須とする設定になっています。
    %
    % 目的: シミュレーションで純粋にMEXを用いるため、MATLAB側のフォールバック実装
    % をコメントアウト（無効化）しました。MEXが存在しない場合は、明示的にビルド
    % してから再度実行してください。
    %
    % もしMEXが存在しない場合は明示的なエラーを出してビルドを促します。
    if use_mex
        try
            [varargout{1:nargout}] = mex_kalman_filter_core(func_name, varargin{:});
            return;
        catch ME
            error('kalman_filter_core:mexFailed', 'MEX call failed: %s\nPlease rebuild MEX by running: cd(fullfile(fileparts(mfilename(''fullpath'')), ''..'', ''cpp'')); build_mex();', ME.message);
        end
    else
        error(['MEX implementation not available. ', ...
            'Please build mex_kalman_filter_core by running (in MATLAB):\n', ...
            '  cd(fullfile(fileparts(mfilename(''fullpath'')), ''..'', ''cpp''))\n', ...
            '  build_mex()\n', ...
            'Then clear persistent variables: clear kalman_filter_core\n', ...
            'And re-run your simulation.']);
    end

% NOTE: MATLAB implementations were intentionally removed/commented-out to force
% use of the MEX implementation. If you want to restore the MATLAB fallback,
% revert this file to the previous version.
