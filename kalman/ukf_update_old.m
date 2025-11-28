function [x_upd, P_upd, K, S, y] = ukf_update(x, P, z, h_func, R, alpha, beta, kappa)
    % UKF_UPDATE  Unscented Kalman Filter縺ｫ繧医ｋ隕ｳ貂ｬ譖ｴ譁ｰ
    % C++螳溯｣・′蛻ｩ逕ｨ蜿ｯ閭ｽ縺ｪ蝣ｴ蜷医・閾ｪ蜍慕噪縺ｫMEX繧剃ｽｿ逕ｨ
    %
    % 蜈･蜉・
    %   x       - 莠句燕謗ｨ螳夂憾諷九・繧ｯ繝医Ν (nx1)
    %   P       - 莠句燕謗ｨ螳壼・蛻・淵陦悟・ (nxn)
    %   z       - 隕ｳ貂ｬ蛟､ (mx1)
    %   h_func  - 隕ｳ貂ｬ髢｢謨ｰ繝上Φ繝峨Ν: z_pred = h_func(x_sig)
    %   R       - 隕ｳ貂ｬ繝弱う繧ｺ蜈ｱ蛻・淵陦悟・ (mxm)
    %   alpha   - UKF繧ｹ繧ｱ繝ｼ繝ｪ繝ｳ繧ｰ繝代Λ繝｡繝ｼ繧ｿ (optional, default: 0.1)
    %   beta    - UKF蛻・ｸ・ヱ繝ｩ繝｡繝ｼ繧ｿ (optional, default: 2)
    %   kappa   - UKF莠梧ｬ｡繝代Λ繝｡繝ｼ繧ｿ (optional, default: 0)
    %
    % 蜃ｺ蜉・
    %   x_upd   - 莠句ｾ梧耳螳夂憾諷九・繧ｯ繝医Ν (nx1)
    %   P_upd   - 莠句ｾ梧耳螳壼・蛻・淵陦悟・ (nxn)
    %   K       - 繧ｫ繝ｫ繝槭Φ繧ｲ繧､繝ｳ (nxm)
    %   S       - 繧､繝弱・繝ｼ繧ｷ繝ｧ繝ｳ蜈ｱ蛻・淵 (mxm)
    %   y       - 繧､繝弱・繝ｼ繧ｷ繝ｧ繝ｳ (mx1)

    if nargin < 6, alpha = 1e-3; end
    if nargin < 7, beta = 2; end
    if nargin < 8, kappa = 0; end
    
    % C++螳溯｣・ｒ蜆ｪ蜈育噪縺ｫ菴ｿ逕ｨ
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
    
    % MATLAB螳溯｣・ｼ医ヵ繧ｩ繝ｼ繝ｫ繝舌ャ繧ｯ・・
    % 繧ｷ繧ｰ繝槭・繧､繝ｳ繝医→驥阪∩縺ｮ逕滓・
    [sig, wm, wc] = ukf_sigma_points(x, P, alpha, beta, kappa);

    % 蜷・す繧ｰ繝槭・繧､繝ｳ繝医ｒ隕ｳ貂ｬ繝｢繝・Ν縺ｧ螟画鋤
    n_sig = size(sig, 2);
    z_pred = zeros(length(z), n_sig);
    for i = 1:n_sig
        z_pred(:, i) = h_func(sig(:, i));
    end

    % 莠域ｸｬ隕ｳ貂ｬ蛟､縺ｮ蟷ｳ蝮・    z_mean = z_pred * wm;

    % 繧､繝弱・繝ｼ繧ｷ繝ｧ繝ｳ蜈ｱ蛻・淵 S 縺ｨ繧ｯ繝ｭ繧ｹ蜈ｱ蛻・淵 Pxz 縺ｮ險育ｮ・    m = length(z);
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

    % 繧ｫ繝ｫ繝槭Φ繧ｲ繧､繝ｳ縺ｮ險育ｮ・    K = Pxz / S;

    % 繧､繝弱・繝ｼ繧ｷ繝ｧ繝ｳ
    y = z - z_mean;

    % 迥ｶ諷九→蜈ｱ蛻・淵縺ｮ譖ｴ譁ｰ
    x_upd = x + K * y;
    
    % 讓呎ｺ也噪縺ｪ蜈ｱ蛻・淵譖ｴ譁ｰ・域焚蛟､螳牙ｮ夂沿・・    % P_upd = P - K*S*K' 縺ｯ逅・ｫ也噪縺ｫ縺ｯ豁｣縺励＞縺後∵焚蛟､逧・↓荳榊ｮ牙ｮ・    % 繧医ｊ螳牙ｮ壹↑蠖｢蠑上ｒ菴ｿ逕ｨ
    P_upd = P - K * S * K';
    
    % 蟇ｾ遘ｰ諤ｧ繧剃ｿ晁ｨｼ
    P_upd = (P_upd + P_upd') / 2;
    
    % 豁｣螳壼､諤ｧ繧剃ｿ晁ｨｼ・亥ｿ・ｦ√↑蝣ｴ蜷医・縺ｿ・・    try
        chol(P_upd);
    catch
        % 繧ｳ繝ｬ繧ｹ繧ｭ繝ｼ蛻・ｧ｣螟ｱ謨暦ｼ壽ｭ｣螳壼､縺ｧ縺ｪ縺・        min_eig = min(eig(P_upd));
        if min_eig < 1e-12
            P_upd = P_upd + eye(n) * (1e-9 - min_eig);
        end
    end

end
