function [x_upd, P_upd, K, S, y] = ukf_update(x, P, z, h_func, R, alpha, beta, kappa)
    % UKF_UPDATE  Unscented Kalman Filterによる観測更新
    %
    % 入力:
    %   x       - 事前推定状態ベクトル (nx1)
    %   P       - 事前推定共分散行列 (nxn)
    %   z       - 観測値 (mx1)
    %   h_func  - 観測関数ハンドル: z_pred = h_func(x_sig)
    %   R       - 観測ノイズ共分散行列 (mxm)
    %   alpha   - UKFスケーリングパラメータ (optional, default: 1e-3)
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
    P_upd = P - K * S * K';

    % 共分散の対称性を保証
    P_upd = (P_upd + P_upd') / 2;

end
