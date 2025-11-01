classdef SensorFilter
    % SENSORFILTER  センサー観測のフィルタリングクラス
    %
    % イノベーションのゲーティング（外れ値除去）を行う
    
    properties (Constant)
        SIGMA_THRESHOLD = 1.0;  % σ閾値（1σでフィルタリング）
    end
    
    methods (Static)
        function [y_filtered, should_update] = filterInnovation(y, R)
            % FILTERINNOVATION  イノベーションをフィルタリング
            %
            % 入力:
            %   y - イノベーション
            %   R - 観測ノイズ共分散
            %
            % 出力:
            %   y_filtered   - フィルタリング済みイノベーション
            %   should_update - 更新すべきかどうか (true/false)
            
            % デフォルト
            y_filtered = y;
            should_update = true;
            
            % R から標準偏差を計算
            if isscalar(R)
                sigma = sqrt(R);
            else
                sigma = sqrt(diag(R));
            end
            
            % σ閾値を超えるイノベーションは棄却
            threshold = SensorFilter.SIGMA_THRESHOLD * sigma;
            
            % 各成分をチェック
            outlier_mask = abs(y) > threshold;
            
            % 一つでも外れ値があれば更新しない
            if any(outlier_mask)
                should_update = false;
                return;
            end
            
            % すべて正常範囲内なら更新
            y_filtered = y;
        end
        
        function snr = computeSNR(y, R)
            % COMPUTESNR  イノベーションのSNRを計算
            %
            % 入力:
            %   y - イノベーション
            %   R - 観測ノイズ共分散
            %
            % 出力:
            %   snr - Signal-to-Noise Ratio
            
            if isscalar(R)
                sigma = sqrt(R);
            else
                sigma = sqrt(diag(R));
            end
            
            % SNR = |signal| / noise
            snr = abs(y) ./ sigma;
        end
    end
end
