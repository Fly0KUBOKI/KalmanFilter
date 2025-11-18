classdef OutlierDetector
    % OUTLIERDETECTOR 外れ値検出の静的メソッド集
    % C++移行を想定した設計
    
    methods (Static)
        function is_outlier = check_innovation(innovation, S, threshold)
            % イノベーションの外れ値判定
            % 入力:
            %   innovation: イノベーション [nx1]
            %   S: イノベーション共分散 [nxn]
            %   threshold: マハラノビス距離の閾値
            % 出力:
            %   is_outlier: true/false
            
            mahal_dist = OutlierDetector.mahalanobis_distance(innovation, S);
            is_outlier = mahal_dist > threshold;
        end
        
        function is_outlier = chi_square_test(innovation, S, dof, alpha)
            % カイ二乗検定による外れ値判定
            % 入力:
            %   innovation: イノベーション [nx1]
            %   S: イノベーション共分散 [nxn]
            %   dof: 自由度
            %   alpha: 有意水準 (例: 0.05)
            % 出力:
            %   is_outlier: true/false
            
            mahal_dist_sq = OutlierDetector.mahalanobis_distance_squared(innovation, S);
            
            % カイ二乗分布の閾値
            threshold = chi2inv(1 - alpha, dof);
            
            is_outlier = mahal_dist_sq > threshold;
        end
        
        function is_outlier = mahalanobis_test(innovation, S, threshold)
            % マハラノビス距離による外れ値判定
            is_outlier = OutlierDetector.check_innovation(innovation, S, threshold);
        end
        
        function [is_valid, innovation_scaled, scale_factor] = adaptive_gating(innovation, S, max_sigma)
            % 適応的イノベーションゲーティング
            % 大きすぎるイノベーションを圧縮
            % 入力:
            %   innovation: イノベーション [nx1]
            %   S: イノベーション共分散 [nxn]
            %   max_sigma: 最大許容σ (例: 5.0)
            % 出力:
            %   is_valid: 更新を実行すべきか
            %   innovation_scaled: スケーリングされたイノベーション
            %   scale_factor: スケーリング係数
            
            mahal_dist = OutlierDetector.mahalanobis_distance(innovation, S);
            
            if mahal_dist > max_sigma
                % イノベーションを圧縮
                scale_factor = max_sigma / mahal_dist;
                innovation_scaled = innovation * scale_factor;
                is_valid = true;
            else
                scale_factor = 1.0;
                innovation_scaled = innovation;
                is_valid = true;
            end
        end
        
        function [is_valid, innovation_compressed] = innovation_compression(innovation, S, max_sigma, reject_sigma)
            % イノベーション圧縮（ArduPilot風）
            % 入力:
            %   innovation: イノベーション
            %   S: 共分散
            %   max_sigma: 圧縮開始閾値 (例: 5.0)
            %   reject_sigma: 拒否閾値 (例: 10.0)
            % 出力:
            %   is_valid: 更新すべきか
            %   innovation_compressed: 圧縮されたイノベーション
            
            mahal_dist = OutlierDetector.mahalanobis_distance(innovation, S);
            
            if mahal_dist > reject_sigma
                % 完全に拒否
                is_valid = false;
                innovation_compressed = innovation;
            elseif mahal_dist > max_sigma
                % 圧縮
                compression = max_sigma / mahal_dist;
                innovation_compressed = innovation * compression;
                is_valid = true;
            else
                % そのまま使用
                innovation_compressed = innovation;
                is_valid = true;
            end
        end
        
        %% ヘルパー関数
        function dist = mahalanobis_distance(innovation, S)
            % マハラノビス距離の計算
            % dist = sqrt(innovation' * inv(S) * innovation)
            
            try
                dist_sq = OutlierDetector.mahalanobis_distance_squared(innovation, S);
                dist = sqrt(dist_sq);
            catch
                % 特異行列の場合
                dist = inf;
            end
        end
        
        function dist_sq = mahalanobis_distance_squared(innovation, S)
            % マハラノビス距離の二乗
            
            % 数値安定性のため正則化
            S_reg = S + eye(size(S)) * 1e-12;
            
            try
                S_inv = inv(S_reg);
                dist_sq = innovation' * S_inv * innovation;
            catch
                dist_sq = inf;
            end
        end
        
        function is_outlier = simple_threshold_test(value, threshold)
            % 単純な閾値テスト
            is_outlier = abs(value) > threshold;
        end
        
        function is_outlier = vector_norm_test(vec, threshold)
            % ベクトルノルムによる閾値テスト
            is_outlier = norm(vec) > threshold;
        end
    end
end
