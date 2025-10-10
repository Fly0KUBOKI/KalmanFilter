classdef UKF_Calculator < handle
% UKF_CALCULATOR - Unscented Kalman Filter 計算クラス
% シンプルで安定化した UKF 実装（シグマポイント、予測、更新）

properties (Access = private)
    n_states
    n_measurements
    alpha = 0.001
    beta = 2
    kappa = 3
    lambda
    wm
    wc
end

methods
    function obj = UKF_Calculator(n_states, n_measurements)
        obj.n_states = n_states;
        obj.n_measurements = n_measurements;
        obj.lambda = obj.alpha^2 * (n_states + obj.kappa) - n_states;

        Nsp = 2 * n_states + 1;
        obj.wm = zeros(Nsp,1);
        obj.wc = zeros(Nsp,1);
        obj.wm(1) = obj.lambda / (n_states + obj.lambda);
        obj.wc(1) = obj.wm(1) + (1 - obj.alpha^2 + obj.beta);
        for j = 2:Nsp
            obj.wm(j) = 1 / (2 * (n_states + obj.lambda));
            obj.wc(j) = obj.wm(j);
        end
    end

    function [x_pred, P_pred] = predict(obj, x_prev, P_prev, F, Q, ~)
        % シグマポイント生成
        sigma_points = obj.generate_sigma_points(x_prev, P_prev);
        Nsp = size(sigma_points,2);

        % シグマポイント伝播
        sigma_pred = zeros(obj.n_states, Nsp);
        for j = 1:Nsp
            sigma_pred(:,j) = F * sigma_points(:,j);
        end

        % 予測平均
        x_pred = sigma_pred * obj.wm;

        % 予測共分散
        P_pred = Q;
        for j = 1:Nsp
            d = sigma_pred(:,j) - x_pred;
            P_pred = P_pred + obj.wc(j) * (d * d');
        end

        % 対称化・正定値化
        P_pred = 0.5 * (P_pred + P_pred');
        P_pred = P_pred + eye(size(P_pred)) * 1e-8;
    end

    function [x_upd, P_upd, y, S, K] = update(obj, x_pred, P_pred, z, H, R)
        % シグマポイント生成（予測から）
        sigma_points = obj.generate_sigma_points(x_pred, P_pred);
        Nsp = size(sigma_points,2);

        % 観測予測
        z_pred = zeros(obj.n_measurements, Nsp);
        for j = 1:Nsp
            z_pred(:,j) = H * sigma_points(:,j);
        end
        z_mean = z_pred * obj.wm;

        % イノベーション共分散
        S = R;
        for j = 1:Nsp
            dz = z_pred(:,j) - z_mean;
            S = S + obj.wc(j) * (dz * dz');
        end
        S = 0.5 * (S + S');

        % クロス共分散
        Pxz = zeros(obj.n_states, obj.n_measurements);
        for j = 1:Nsp
            dx = sigma_points(:,j) - x_pred;
            dz = z_pred(:,j) - z_mean;
            Pxz = Pxz + obj.wc(j) * (dx * dz');
        end

        % カルマンゲイン
        K = Pxz / S;

        % イノベーション
        y = z - z_mean;
        innov_norm = norm(y);
        expected_scale = sqrt(max(trace(S), eps));
        outlier_thresh = 20 * expected_scale;
        if innov_norm > outlier_thresh
            % 詳細ログ出力: 観測と予測観測、Sの対角を出す
            try
                z_print = sprintf(' %.3g', z(1:min(end,6))');
            catch
                z_print = '<unable to format z>';
            end
            try
                zm_print = sprintf(' %.3g', z_mean(1:min(end,6))');
            catch
                zm_print = '<unable to format z_mean>';
            end
            sd = diag(S);
            try
                sd_print = sprintf(' %.3g', sd(1:min(end,6))');
            catch
                sd_print = '<unable to format S diag>';
            end
            fprintf('UKF_SKIPPED_UPDATE: innov_norm=%.3g thresh=%.3g z=%s z_mean=%s S_diag=%s\n', innov_norm, outlier_thresh, z_print, zm_print, sd_print);
            warning('UKF_Calculator.update: large innovation detected (%.3g > %.3g). Skipping update.', innov_norm, outlier_thresh);
            x_upd = x_pred;
            P_upd = 0.5 * (P_pred + P_pred') + eye(size(P_pred)) * 1e-6;
            return;
        end

        % 更新
        x_upd = x_pred + K * y;
        P_upd = P_pred - K * S * K';
        P_upd = 0.5 * (P_upd + P_upd');
        P_upd = P_upd + eye(size(P_upd)) * 1e-6;
    end

    function [x_upd, P_upd, y, S, K] = predict_and_update(obj, x_prev, P_prev, F, Q, dt, z, H, R)
        [x_pred, P_pred] = obj.predict(x_prev, P_prev, F, Q, dt);
        [x_upd, P_upd, y, S, K] = obj.update(x_pred, P_pred, z, H, R);
    end
end

methods (Access = private)
    function sigma_points = generate_sigma_points(obj, x, P)
        n = obj.n_states;
        Nsp = 2*n + 1;
        sigma_points = zeros(n, Nsp);

        % スケーリングされた共分散の平方根
        S = (n + obj.lambda) * P;
        try
            sqrt_S = chol(S, 'lower');
        catch
            [V,D] = eig(S);
            D = diag(max(diag(D), 0));
            sqrt_S = V * sqrt(D);
        end

        sigma_points(:,1) = x;
        for j = 1:n
            sigma_points(:, j+1)     = x + sqrt_S(:, j);
            sigma_points(:, n+j+1) = x - sqrt_S(:, j);
        end
    end
end

end