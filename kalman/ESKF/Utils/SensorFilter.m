classdef SensorFilter
    % SENSORFILTER  センサー観測のフィルタリング共通クラス
    %
    % 2σ閾値によるイノベーションフィルタリングを提供
    
    properties (Constant)
        SIGMA_THRESHOLD = 2.0;  % 2σ閾値
    end
    
    methods (Static)
        function [filtered_innovation, should_update] = filterInnovation(innovation, R_noise)
            % FILTERINNOVATION  イノベーションを2σ閾値でフィルタリング
            %
            % 入力:
            %   innovation - イノベーションベクトル (nx1)
            %   R_noise    - 観測ノイズ共分散の対角要素 (nx1) または (nxn)
            %
            % 出力:
            %   filtered_innovation - フィルタリング後のイノベーション
            %   should_update       - 更新すべきかどうか (boolean)
            
            filtered_innovation = innovation;
            n = length(innovation);
            
            % R_noiseが行列の場合は対角要素を取得
            if size(R_noise, 1) == size(R_noise, 2) && size(R_noise, 1) > 1
                R_diag = diag(R_noise);
            else
                R_diag = R_noise(:);
            end
            
            % 各要素ごとに2σ閾値でフィルタリング
            for i = 1:n
                noise_std = sqrt(max(R_diag(i), eps));
                threshold = SensorFilter.SIGMA_THRESHOLD * noise_std;
                
                if abs(innovation(i)) < threshold
                    filtered_innovation(i) = 0;
                end
            end
            
            % 全てのイノベーションが0なら更新不要
            should_update = (norm(filtered_innovation) > eps);
        end
        
        function should_update = shouldUpdate(innovation, R_noise)
            % SHOULDUPDATE  更新すべきかどうかを判定
            %
            % 入力:
            %   innovation - イノベーションベクトル (nx1)
            %   R_noise    - 観測ノイズ共分散の対角要素 (nx1)
            %
            % 出力:
            %   should_update - 更新すべきかどうか (boolean)
            
            [~, should_update] = SensorFilter.filterInnovation(innovation, R_noise);
        end
        
        function filtered_dx = filterStateCorrection(dx, R_noise, H, component_indices)
            % FILTERSTATECORRECTION  状態修正量をフィルタリング
            %
            % 入力:
            %   dx                - 状態修正量ベクトル (15x1)
            %   R_noise          - 観測ノイズ共分散
            %   H                - 観測行列
            %   component_indices - フィルタ対象の状態インデックス
            %
            % 出力:
            %   filtered_dx - フィルタリング後の状態修正量
            
            filtered_dx = dx;
            
            if nargin < 4 || isempty(component_indices)
                return;
            end
            
            % R_noiseから標準偏差を取得
            if size(R_noise, 1) == size(R_noise, 2)
                R_diag = diag(R_noise);
            else
                R_diag = R_noise(:);
            end
            
            % 各成分について閾値判定
            for i = 1:length(component_indices)
                idx = component_indices(i);
                
                % 観測行列から感度を推定
                if ~isempty(H) && size(H, 2) >= idx
                    sensitivity = max(norm(H(:, idx)), eps);
                    noise_std = sqrt(max(mean(R_diag), eps));
                    threshold = SensorFilter.SIGMA_THRESHOLD * noise_std / sensitivity;
                else
                    % デフォルト閾値
                    threshold = 0.001;
                end
                
                if abs(filtered_dx(idx)) < threshold
                    filtered_dx(idx) = 0;
                end
            end
        end
    end
end
