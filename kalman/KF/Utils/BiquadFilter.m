classdef BiquadFilter < handle
    % BIQUADFILTER  2次IIRフィルタ（Biquad）
    %
    % 低遅延でスパイクを除去するための2次ローパスフィルタ
    % EMAより位相特性が良好で、カットオフ周波数を明示的に設定可能
    
    properties
        % フィルタ係数
        b0, b1, b2  % 分子係数
        a1, a2      % 分母係数（a0=1で正規化済み）
        
        % 状態変数（Direct Form II）
        x1, x2      % 入力履歴
        y1, y2      % 出力履歴
        
        % フィルタ設定
        sample_rate   % サンプリング周波数 [Hz]
        cutoff_freq   % カットオフ周波数 [Hz]
    end
    
    methods
        function obj = BiquadFilter(sample_rate, cutoff_freq)
            % コンストラクタ
            %
            % 入力:
            %   sample_rate  - サンプリング周波数 [Hz] (例: 200)
            %   cutoff_freq  - カットオフ周波数 [Hz] (例: 20)
            
            obj.sample_rate = sample_rate;
            obj.cutoff_freq = cutoff_freq;
            
            % 状態変数初期化
            obj.x1 = 0;
            obj.x2 = 0;
            obj.y1 = 0;
            obj.y2 = 0;
            
            % フィルタ係数を計算
            obj.computeCoefficients();
        end
        
        function computeCoefficients(obj)
            % 2次バターワースローパスフィルタの係数を計算
            %
            % デジタルフィルタ設計:
            %   H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
            
            % 正規化周波数
            omega = 2 * pi * obj.cutoff_freq / obj.sample_rate;
            
            % プリワーピング（双1次変換の周波数歪み補正）
            K = tan(omega / 2);
            K2 = K * K;
            
            % 2次バターワース（Q = 1/sqrt(2) = 0.7071）
            Q = 1 / sqrt(2);
            
            % 正規化（a0 = 1）
            norm = 1 + K / Q + K2;
            
            % 分子係数（ローパス）
            obj.b0 = K2 / norm;
            obj.b1 = 2 * obj.b0;
            obj.b2 = obj.b0;
            
            % 分母係数
            obj.a1 = 2 * (K2 - 1) / norm;
            obj.a2 = (1 - K / Q + K2) / norm;
        end
        
        function y = apply(obj, x)
            % フィルタを適用（単一サンプル）
            %
            % 入力:
            %   x - 入力サンプル（スカラー）
            %
            % 出力:
            %   y - フィルタ済み出力（スカラー）
            
            % Direct Form II 構造
            % w[n] = x[n] - a1*w[n-1] - a2*w[n-2]
            % y[n] = b0*w[n] + b1*w[n-1] + b2*w[n-2]
            
            w = x - obj.a1 * obj.x1 - obj.a2 * obj.x2;
            y = obj.b0 * w + obj.b1 * obj.x1 + obj.b2 * obj.x2;
            
            % 状態更新
            obj.x2 = obj.x1;
            obj.x1 = w;
            obj.y2 = obj.y1;
            obj.y1 = y;
        end
        
        function reset(obj)
            % フィルタ状態をリセット
            obj.x1 = 0;
            obj.x2 = 0;
            obj.y1 = 0;
            obj.y2 = 0;
        end
        
        function setCutoffFreq(obj, cutoff_freq)
            % カットオフ周波数を変更
            obj.cutoff_freq = cutoff_freq;
            obj.computeCoefficients();
        end
    end
end
