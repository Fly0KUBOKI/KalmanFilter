function [dtheta, P_upd, K, S, y] = meukf_update_attitude(P_sub, q_nom, z, h_func, R, alpha, beta, kappa)
    % MEUKF_UPDATE_ATTITUDE  Manifold Error UKFによる姿勢更新
    % 誤差状態空間(3D回転ベクトル)でシグマ点を生成し、
    % クォータニオン多様体上で観測モデルを評価する
    %
    % 入力:
    %   P_sub   - 誤差状態の共分散 (3x3, 姿勢誤差部分のみ)
    %   q_nom   - ノミナル姿勢クォータニオン (4x1)
    %   z       - 観測値 (mx1)
    %   h_func  - 観測関数ハンドル: z_pred = h_func(q)
    %   R       - 観測ノイズ共分散 (mxm)
    %   alpha   - UKFパラメータ (default: 1e-3)
    %   beta    - UKFパラメータ (default: 2)
    %   kappa   - UKFパラメータ (default: 0)
    %
    % 出力:
    %   dtheta  - 姿勢誤差修正量 (3x1, 回転ベクトル)
    %   P_upd   - 更新後の誤差共分散 (3x3)
    %   K       - カルマンゲイン (3xm)
    %   S       - イノベーション共分散 (mxm)
    %   y       - イノベーション (mx1)

    if nargin < 6, alpha = 1e-3; end
    if nargin < 7, beta = 2; end
    if nargin < 8, kappa = 0; end
    
    n = 3;  % 誤差状態の次元 (回転ベクトル)
    m = length(z);
    
    % 入力検証
    if any(isnan(P_sub(:))) || any(isinf(P_sub(:)))
        error('P_sub contains NaN or Inf');
    end
    if any(isnan(z(:))) || any(isinf(z(:)))
        error('Observation z contains NaN or Inf');
    end
    
    % UKFパラメータ計算
    lambda = alpha^2 * (n + kappa) - n;
    gamma = sqrt(n + lambda);
    
    % 重み計算
    Wm = zeros(2*n+1, 1);
    Wc = zeros(2*n+1, 1);
    Wm(1) = lambda / (n + lambda);
    Wc(1) = Wm(1) + (1 - alpha^2 + beta);
    for i = 2:(2*n+1)
        Wm(i) = 1 / (2 * (n + lambda));
        Wc(i) = Wm(i);
    end
    
    % 誤差空間でシグマ点生成 (0周辺)
    % P_subの平方根 (Cholesky)
    % 正定値化
    P_sub = (P_sub + P_sub') / 2;  % 対称化
    min_eig = min(eig(P_sub));
    if min_eig <= 0
        % 正定値でない場合は正則化
        P_sub = P_sub + eye(n) * (abs(min_eig) + 1e-6);
    end
    
    % Cholesky分解
    try
        L = chol(P_sub, 'lower');
    catch ME
        % フォールバック: より強い正則化
        P_sub = P_sub + eye(n) * 1e-4;
        try
            L = chol(P_sub, 'lower');
        catch
            error('Cholesky decomposition failed even after regularization: %s', ME.message);
        end
    end
    
    % 誤差シグマ点 (3D回転ベクトル)
    dtheta_sig = zeros(n, 2*n+1);
    dtheta_sig(:,1) = zeros(n,1);  % 中心点
    for i = 1:n
        dtheta_sig(:, i+1) = gamma * L(:,i);
        dtheta_sig(:, i+n+1) = -gamma * L(:,i);
    end
    
    % 各シグマ点を姿勢空間に写像して観測モデル評価
    Z_sig = zeros(m, 2*n+1);
    for i = 1:(2*n+1)
        % 誤差クォータニオン: dq = exp(dtheta/2)
        dq = QuaternionLib.small_angle_quat(dtheta_sig(:,i));
        % 全体クォータニオン: q = q_nom ⊗ dq
        q_full = QuaternionLib.multiply(q_nom, dq);
        q_full = QuaternionLib.normalize(q_full);
        % 観測予測
        z_pred = h_func(q_full);
        % 列ベクトルに変換して格納
        Z_sig(:,i) = z_pred(:);
    end
    
    % 観測平均
    z_mean = Z_sig * Wm;
    
    % イノベーション共分散 S と クロス共分散 Pxz
    S = R;
    Pxz = zeros(n, m);
    for i = 1:(2*n+1)
        dz = Z_sig(:,i) - z_mean;
        S = S + Wc(i) * (dz * dz');
        Pxz = Pxz + Wc(i) * (dtheta_sig(:,i) * dz');
    end
    
    % S対称化
    S = (S + S') / 2;
    
    % イノベーション (列ベクトルに統一)
    y = z(:) - z_mean(:);
    
    % カルマンゲイン
    try
        K = Pxz / S;
    catch
        % フォールバック: Cholesky
        try
            U = chol(S);
            K = (U \ (U' \ Pxz'))';
        catch
            % 最終手段: 擬似逆行列
            K = Pxz * pinv(S);
        end
    end
    
    % 姿勢誤差修正量
    dtheta = K * y;
    
    % 共分散更新 (Joseph form)
    % 観測行列の近似: Pxz ≈ P_sub * H'より H ≈ inv(P_sub) * Pxz (近似)
    % 簡略化: I - K*H ≈ I - K*(Pxz'/P_sub) だが、数値的に不安定
    % 代わりに安定な形: P_upd = P_sub - K*S*K' + K*R*K'
    P_upd = P_sub - K * S * K' + K * R * K';
    P_upd = (P_upd + P_upd') / 2;
end
