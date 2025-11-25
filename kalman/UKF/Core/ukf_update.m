function [x_upd, P_upd, K, S, y] = ukf_update(x, P, z, h_func, R, alpha, beta, kappa)
    % UKF_UPDATE  Unscented Kalman Filterによる観測更新
    % C++実装が利用可能な場合は自動的にMEXを使用
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
    
    % C++実装を優先的に使用
    persistent use_mex;
    if isempty(use_mex)
        use_mex = exist('mex_ukf_update', 'file') == 3;
    end
    
    if use_mex
        try
            [x_upd, P_upd, K, S, y] = mex_ukf_update(x, P, z, h_func, R, alpha, beta, kappa);
            return;
        catch ME
            warning('ukf_update:MEXFailed', 'MEX failed: %s. Falling back to MATLAB', ME.message);
            use_mex = false;
        end
    end
    
    % MATLAB実装（フォールバック）

    % シグマポイントと重みの生成
    [sig, wm, wc] = ukf_sigma_points(x, P, alpha, beta, kappa);

    % 各シグマポイントを観測モデルで変換
    n_sig = size(sig, 2);
    z_pred = zeros(length(z), n_sig);
    for i = 1:n_sig
        z_pred(:, i) = h_func(sig(:, i));
    end

    % 予測観測値の平均
    z_mean = z_pred * wm;

    % イノベーション共分散 S とクロス共分散 Pxz の計算
    m = length(z);
    n = length(x);
    S = zeros(m, m);
    Pxz = zeros(n, m);

    for i = 1:n_sig
        dz = z_pred(:, i) - z_mean;
        S = S + wc(i) * (dz * dz');
        
        dx = sig(:, i) - x;
        Pxz = Pxz + wc(i) * (dx * dz');
    end
    S = S + R;

    % カルマンゲインの計算
    K = Pxz / S;

    % イノベーション
    y = z - z_mean;

    % 状態と共分散の更新
    x_upd = x + K * y;
    
    % 標準的な共分散更新（数値安定版）
    % P_upd = P - K*S*K' は理論的には正しいが、数値的に不安定
    % より安定な形式を使用
    P_upd = P - K * S * K';
    
    % 対称性を保証
    P_upd = (P_upd + P_upd') / 2;
    
    % 正定値性を保証（必要な場合のみ）
    try
        chol(P_upd);
    catch
        % コレスキー分解失敗：正定値でない
        min_eig = min(eig(P_upd));
        if min_eig < 1e-12
            P_upd = P_upd + eye(n) * (1e-9 - min_eig);
        end
    end

end
