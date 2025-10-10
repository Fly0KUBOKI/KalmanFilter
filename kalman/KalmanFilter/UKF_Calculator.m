classdef UKF_Calculator < handle
% UKF_CALCULATOR - Unscented Kalman Filter計算クラス
% 各推定ブロックから独立したインスタンスを作成可能

properties (Access = private)
    n_states           % 状態変数の数
    n_measurements     % 観測変数の数
    alpha = 1e-3       % UKFパラメータ
    beta = 2           % UKFパラメータ
    kappa = 0          % UKFパラメータ
    lambda             % 複合パラメータ
    wm                 % 平均重み
    wc                 % 共分散重み
end

methods
    function obj = UKF_Calculator(n_states, n_measurements)
        % コンストラクタ
        obj.n_states = n_states;
        obj.n_measurements = n_measurements;
        
        % UKFパラメータ計算
        obj.lambda = obj.alpha^2 * (n_states + obj.kappa) - n_states;
        
        % 重み計算
        obj.wm = zeros(2*n_states + 1, 1);
        obj.wc = zeros(2*n_states + 1, 1);
        
        obj.wm(1) = obj.lambda / (n_states + obj.lambda);
        obj.wc(1) = obj.wm(1) + (1 - obj.alpha^2 + obj.beta);
        
        for i = 2:(2*n_states + 1)
            obj.wm(i) = 1 / (2 * (n_states + obj.lambda));
            obj.wc(i) = obj.wm(i);
        end
    end
    
    function [x_pred, P_pred] = predict(obj, x_prev, P_prev, F, Q, ~)
        % UKF予測ステップ
        
        % シグマポイント生成
        sigma_points = obj.generate_sigma_points(x_prev, P_prev);
        
        % シグマポイント伝播
        sigma_pred = zeros(obj.n_states, 2*obj.n_states + 1);
        for i = 1:(2*obj.n_states + 1)
            sigma_pred(:, i) = F * sigma_points(:, i);
        end
        
        % 予測平均
        x_pred = sigma_pred * obj.wm;
        
        % 予測共分散
        P_pred = Q;
        for i = 1:(2*obj.n_states + 1)
            diff = sigma_pred(:, i) - x_pred;
            P_pred = P_pred + obj.wc(i) * (diff * diff');
        end
        
        % 数値安定性のための対称化
        P_pred = 0.5 * (P_pred + P_pred');
    end
    
    function [x_upd, P_upd, y, S, K] = update(obj, x_pred, P_pred, z, H, R)
        % UKF更新ステップ
        
        % シグマポイント生成
        sigma_points = obj.generate_sigma_points(x_pred, P_pred);
        
        % 観測予測
        z_pred = zeros(obj.n_measurements, 2*obj.n_states + 1);
        for i = 1:(2*obj.n_states + 1)
            z_pred(:, i) = H * sigma_points(:, i);
        end
        
        % 観測予測平均
        z_mean = z_pred * obj.wm;
        
        % イノベーション共分散
        S = R;
        for i = 1:(2*obj.n_states + 1)
            diff_z = z_pred(:, i) - z_mean;
            S = S + obj.wc(i) * (diff_z * diff_z');
        end
        
        % クロス共分散
        Pxz = zeros(obj.n_states, obj.n_measurements);
        for i = 1:(2*obj.n_states + 1)
            diff_x = sigma_points(:, i) - x_pred;
            diff_z = z_pred(:, i) - z_mean;
            Pxz = Pxz + obj.wc(i) * (diff_x * diff_z');
        end
        
        % カルマンゲイン
        K = Pxz / S;
        
        % イノベーション
        y = z - z_mean;
        
        % 状態更新
        x_upd = x_pred + K * y;
        
        % 共分散更新
        P_upd = P_pred - K * S * K';
        
        % 数値安定性のための対称化
        P_upd = 0.5 * (P_upd + P_upd');
    end
    
    function [x_upd, P_upd, y, S, K] = predict_and_update(obj, x_prev, P_prev, F, Q, dt, z, H, R)
        % 予測と更新を連続実行
        [x_pred, P_pred] = obj.predict(x_prev, P_prev, F, Q, dt);
        [x_upd, P_upd, y, S, K] = obj.update(x_pred, P_pred, z, H, R);
    end
end

methods (Access = private)
    function sigma_points = generate_sigma_points(obj, x, P)
        % シグマポイント生成
        n = obj.n_states;
        
        try
            % Cholesky分解
            sqrt_P = chol((n + obj.lambda) * P, 'lower');
        catch
            % 分解失敗時は固有値分解を使用
            [V, D] = eig((n + obj.lambda) * P);
            sqrt_P = V * sqrt(max(D, 0));
        end
        
        sigma_points = zeros(n, 2*n + 1);
        sigma_points(:, 1) = x;
        
        for i = 1:n
            sigma_points(:, i+1) = x + sqrt_P(:, i);
            sigma_points(:, n+i+1) = x - sqrt_P(:, i);
        end
    end
end

end