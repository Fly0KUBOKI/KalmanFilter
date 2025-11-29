function [sig, wm, wc] = ukf_sigma_points(x, P, alpha, beta, kappa)
    % UKF_SIGMA_POINTS  Unscented Kalman Filterのシグマポイントと重みを生成
    %
    % MEX高速化対応: mex_ukf_sigma_pointsが利用可能な場合は自動的に使用
    %
    % 入力:
    %   x     - 状態ベクトル (nx1)
    %   P     - 共分散行列 (nxn)
    %   alpha - スケーリングパラメータ (default: 1e-3)
    %   beta  - 分布パラメータ (default: 2 for Gaussian)
    %   kappa - 二次パラメータ (default: 0)
    %
    % 出力:
    %   sig   - シグマポイント (nx(2n+1))
    %   wm    - 平均計算用の重み ((2n+1)x1)
    %   wc    - 共分散計算用の重み ((2n+1)x1)

    persistent use_mex;
    
    if nargin < 3, alpha = 1e-3; end
    if nargin < 4, beta = 2; end
    if nargin < 5, kappa = 0; end

    % 初回呼び出し時にMEXファイルの存在をチェック
    if isempty(use_mex)
        use_mex = exist('mex_ukf_sigma_points', 'file') == 3;
        if use_mex
            fprintf('[ukf_sigma_points] MEX acceleration enabled\n');
        end
    end
    
    % MEX実装を使用
    if use_mex
        try
            [sig, wm, wc] = mex_ukf_sigma_points(x, P, alpha, beta, kappa);
            return;
        catch ME
            warning('ukf_sigma_points:mexFallback', 'MEX call failed, falling back to MATLAB: %s', ME.message);
            use_mex = false;
        end
    end
    
    % MATLAB実装（フォールバック）
    n = length(x);
    lambda = alpha^2 * (n + kappa) - n;
    
    % 重みの計算
    wm = zeros(2*n + 1, 1);
    wc = zeros(2*n + 1, 1);
    
    wm(1) = lambda / (n + lambda);
    wc(1) = lambda / (n + lambda) + (1 - alpha^2 + beta);
    
    for i = 2:(2*n + 1)
        wm(i) = 1 / (2 * (n + lambda));
        wc(i) = wm(i);
    end
    
    % シグマポイントの生成
    sig = zeros(n, 2*n + 1);
    sig(:, 1) = x;
    
    try
        L = chol((n + lambda) * P, 'lower');
    catch
        % コレスキー分解失敗時はSVDを使用
        [U, S, ~] = svd(P);
        L = U * sqrt(S) * sqrt(n + lambda);
    end
    
    for i = 1:n
        sig(:, i+1) = x + L(:, i);
        sig(:, i+1+n) = x - L(:, i);
    end

end
