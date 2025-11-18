classdef ESKFMeasurementUpdate
    % ESKFMEASUREMENTUPDATE ESKF測定更新関数集
    % C++移行を想定した設計
    
    methods (Static)
        function [dx, P_new, K, innovation, S] = linear_update(z, h, H, P, R)
            % 線形測定更新
            % 入力:
            %   z: 測定値 [mx1]
            %   h: 予測測定値 [mx1]
            %   H: 観測行列 [mx15]
            %   P: 共分散行列 [15x15]
            %   R: 測定ノイズ共分散 [mxm]
            % 出力:
            %   dx: 誤差状態 [15x1]
            %   P_new: 更新された共分散 [15x15]
            %   K: カルマンゲイン [15xm]
            %   innovation: イノベーション [mx1]
            %   S: イノベーション共分散 [mxm]
            
            % イノベーション計算
            innovation = z - h;
            
            % イノベーション共分散
            S = H * P * H' + R;
            S = CovarianceRegularizer.regularize_innovation_covariance(S);
            
            % カルマンゲイン計算
            K = P * H' / S;
            
            % 誤差状態計算
            dx = K * innovation;
            
            % 共分散更新（Joseph形式）
            P_new = ESKFMeasurementUpdate.joseph_form_covariance_update(P, K, H, R);
        end
        
        function [dx, P_new] = ukf_update(x_err, P, z, h_func, R)
            % UKF測定更新（ESKF用）
            % 入力:
            %   x_err: 誤差状態 [15x1] (通常ゼロ)
            %   P: 共分散行列 [15x15]
            %   z: 測定値 [mx1]
            %   h_func: 観測関数ハンドル h = h_func(dx)
            %   R: 測定ノイズ共分散 [mxm]
            % 出力:
            %   dx: 更新された誤差状態 [15x1]
            %   P_new: 更新された共分散 [15x15]
            
            % UKFパラメータ
            alpha = 1e-3;
            beta = 2.0;
            kappa = 0.0;
            
            n = length(x_err);
            lambda = alpha^2 * (n + kappa) - n;
            
            % シグマポイント生成
            [sigma_points, Wm, Wc] = ESKFMeasurementUpdate.generate_sigma_points(...
                x_err, P, lambda, alpha, beta);
            
            % シグマポイントを観測空間に変換
            m = size(z, 1);
            Z_sigma = zeros(m, 2*n+1);
            for i = 1:(2*n+1)
                Z_sigma(:, i) = h_func(sigma_points(:, i));
            end
            
            % 予測測定値
            z_pred = zeros(m, 1);
            for i = 1:(2*n+1)
                z_pred = z_pred + Wm(i) * Z_sigma(:, i);
            end
            
            % イノベーション共分散
            Pzz = R;
            for i = 1:(2*n+1)
                dz = Z_sigma(:, i) - z_pred;
                Pzz = Pzz + Wc(i) * (dz * dz');
            end
            
            % 状態-測定クロス共分散
            Pxz = zeros(n, m);
            for i = 1:(2*n+1)
                dx_i = sigma_points(:, i) - x_err;
                dz = Z_sigma(:, i) - z_pred;
                Pxz = Pxz + Wc(i) * (dx_i * dz');
            end
            
            % カルマンゲイン
            K = Pxz / Pzz;
            
            % 状態更新
            innovation = z - z_pred;
            dx = x_err + K * innovation;
            
            % 共分散更新
            P_new = P - K * Pzz * K';
            P_new = CovarianceRegularizer.enforce_symmetry(P_new);
        end
        
        function P_new = joseph_form_covariance_update(P, K, H, R)
            % Joseph形式の共分散更新（数値安定性が高い）
            n = size(P, 1);
            I = eye(n);
            IKH = I - K * H;
            
            P_new = IKH * P * IKH' + K * R * K';
            P_new = CovarianceRegularizer.enforce_symmetry(P_new);
        end
        
        function [sigma_points, Wm, Wc] = generate_sigma_points(x, P, lambda, alpha, beta)
            % UKF用シグマポイント生成
            n = length(x);
            sigma_points = zeros(n, 2*n+1);
            
            % Cholesky分解
            try
                L = chol(P, 'lower');
            catch
                % 正則化して再試行
                P_reg = CovarianceRegularizer.regularize_for_ukf(P);
                L = chol(P_reg, 'lower');
            end
            
            % シグマポイント
            sigma_points(:, 1) = x;
            for i = 1:n
                sigma_points(:, i+1) = x + sqrt(n + lambda) * L(:, i);
                sigma_points(:, n+i+1) = x - sqrt(n + lambda) * L(:, i);
            end
            
            % 重み
            Wm = zeros(2*n+1, 1);
            Wc = zeros(2*n+1, 1);
            
            Wm(1) = lambda / (n + lambda);
            Wc(1) = lambda / (n + lambda) + (1 - alpha^2 + beta);
            
            for i = 2:(2*n+1)
                Wm(i) = 1 / (2 * (n + lambda));
                Wc(i) = 1 / (2 * (n + lambda));
            end
        end
    end
end
