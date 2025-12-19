function [x_upd, P_upd, K, S, y] = ukf_update(x, P, z, h_func, R, alpha, beta, kappa)
    % UKF_UPDATE  Unscented Kalman Filterによる観測更新
    % MEX専用実装 - MATLAB fallbackは削除済み
    %
    % 入力:
    %   x       - 事前推定状態ベクトル (nx1)
    %   P       - 事前推定共分散行列 (nxn)
    %   z       - 観測値 (mx1)
    %   h_func  - 観測関数ハンドル: z_pred = h_func(x_sig)
    %   R       - 観測ノイズ共分散行列 (mxm)
    %   alpha   - UKFスケーリングパラメータ (optional, default: 0.1)
    %   beta    - UKF分布パラメータ (optional, default: 2)
    %   kappa   - UKF二次パラメータ (optional, default: 0)
    %
    % 出力:
    %   x_upd   - 事後推定状態ベクトル (nx1)
    %   P_upd   - 事後推定共分散行列 (nxn)
    %   K       - カルマンゲイン (nxm)
    %   S       - イノベーション共分散 (mxm)
    %   y       - イノベーション (mx1)

    if nargin < 6, alpha = 1e-3; end
    if nargin < 7, beta = 2; end
    if nargin < 8, kappa = 0; end
    
    % MEX実装のチェック
    persistent use_mex;
    if isempty(use_mex)
        use_mex = exist('mex_ukf_update', 'file') == 3;
        if ~use_mex
            error('ukf_update:noMEX', 'MEX implementation not found. Please build mex_ukf_update.');
        end
    end
    
    [x_upd, P_upd, K, S, y] = mex_ukf_update(x, P, z, h_func, R, alpha, beta, kappa);
end
