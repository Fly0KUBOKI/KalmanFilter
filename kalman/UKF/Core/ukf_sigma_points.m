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
        % use_mex = exist('mex_ukf_sigma_points', 'file') == 3;
        use_mex = false; % 強制的にMATLAB実装を使用
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
    
    % MATLAB実装
    n = length(x);
    lambda = alpha^2 * (n + kappa) - n;

    % 重みの計算
    wm = [lambda/(n+lambda); repmat(1/(2*(n+lambda)), 2*n, 1)];
    wc = wm;
    wc(1) = wc(1) + (1 - alpha^2 + beta);

    % シグマポイントの生成
    try
        sqrtP = chol((n + lambda) * P, 'lower');
    catch
        % Cholesky分解が失敗した場合、正則化して再試行
        [sqrtP, p] = chol((n + lambda) * (P + eye(n)*1e-9), 'lower');
        if p ~= 0
            warning('ukf_sigma_points: Covariance matrix is not positive definite, using eigenvalue decomposition');
            [V, D] = eig(P);
            D = max(D, 0);  % 負の固有値をゼロに
            sqrtP = V * sqrt((n + lambda) * D);
        end
    end

    sig = [x, repmat(x, 1, n) + sqrtP, repmat(x, 1, n) - sqrtP];

end
