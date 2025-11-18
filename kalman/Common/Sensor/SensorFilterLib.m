classdef SensorFilterLib < handle
    % SENSORFILTERLIB センサーフィルタの統一インターフェース
    %
    % 各センサーに適したフィルタを提供:
    % - 加速度計: EMAフィルタ + 外れ値検出
    % - ジャイロ: Biquadローパスフィルタ
    % - 磁気計: EMAフィルタ + 外れ値検出
    % - GPS: Alpha-Betaフィルタ
    % - 気圧計: EMAフィルタ
    %
    % C++互換設計: 各フィルタは独立したメソッドとして実装
    
    properties
        % フィルタ状態
        accel_state
        gyro_state
        mag_state
        gps_state
        baro_state
    end
    
    methods
        function obj = SensorFilterLib()
            % デフォルト設定で初期化
            obj.accel_state = struct('filtered', zeros(3,1), 'history', [], 'alpha', 0.3);
            obj.gyro_state = struct('x1', zeros(3,1), 'x2', zeros(3,1), 'y1', zeros(3,1), 'y2', zeros(3,1));
            obj.mag_state = struct('filtered', zeros(3,1), 'history', [], 'alpha', 0.2);
            obj.gps_state = struct('filtered', zeros(3,1), 'vel', zeros(3,1), 'alpha', 0.5, 'beta', 0.1);
            obj.baro_state = struct('filtered', 0, 'alpha', 0.4);
        end
        
        %% 加速度計フィルタ
        function [a_filt, is_outlier] = filter_accel(obj, a_meas, a_expected)
            % FILTER_ACCEL 加速度計EMAフィルタ + 外れ値検出
            %
            % 入力:
            %   a_meas     - 計測加速度 [3x1] (m/s^2)
            %   a_expected - 期待加速度 [3x1] (オプション)
            %
            % 出力:
            %   a_filt     - フィルタ済み加速度 [3x1]
            %   is_outlier - 外れ値判定フラグ
            
            if nargin < 3
                a_expected = zeros(3,1);
            end
            
            % 残差
            residual = a_meas - a_expected;
            residual_norm = norm(residual);
            
            % ノイズ推定
            if isempty(obj.accel_state.history)
                noise_std = residual_norm;
            else
                noise_std = std(obj.accel_state.history);
            end
            
            % 外れ値判定 (3σ基準)
            threshold = 3.0 * max(noise_std, 0.1);
            is_outlier = (residual_norm > threshold);
            
            if is_outlier
                % 外れ値の場合は前回値を保持
                a_filt = obj.accel_state.filtered;
            else
                % EMAフィルタ適用
                alpha = obj.accel_state.alpha;
                a_filt = alpha * a_meas + (1 - alpha) * obj.accel_state.filtered;
                obj.accel_state.filtered = a_filt;
                
                % 履歴更新
                obj.accel_state.history = [obj.accel_state.history; residual_norm];
                if length(obj.accel_state.history) > 20
                    obj.accel_state.history = obj.accel_state.history(2:end);
                end
            end
        end
        
        %% ジャイロフィルタ
        function w_filt = filter_gyro(obj, w_meas, dt, cutoff_freq)
            % FILTER_GYRO ジャイロBiquadローパスフィルタ
            %
            % 入力:
            %   w_meas      - 計測角速度 [3x1] (rad/s)
            %   dt          - サンプリング時間 (秒)
            %   cutoff_freq - カットオフ周波数 [Hz] (デフォルト: 20Hz)
            %
            % 出力:
            %   w_filt - フィルタ済み角速度 [3x1]
            
            if nargin < 4
                cutoff_freq = 20;  % デフォルト20Hz
            end
            
            sample_rate = 1.0 / dt;
            
            % Biquad係数を計算（初回または設定変更時のみ）
            if ~isfield(obj.gyro_state, 'fs') || obj.gyro_state.fs ~= sample_rate
                obj.gyro_state.fs = sample_rate;
                obj.gyro_state.fc = cutoff_freq;
                
                % 2次バターワースローパス係数
                omega = 2 * pi * cutoff_freq / sample_rate;
                K = tan(omega / 2);
                Q = 1 / sqrt(2);
                norm = 1 + K/Q + K^2;
                
                obj.gyro_state.b0 = K^2 / norm;
                obj.gyro_state.b1 = 2 * obj.gyro_state.b0;
                obj.gyro_state.b2 = obj.gyro_state.b0;
                obj.gyro_state.a1 = 2 * (K^2 - 1) / norm;
                obj.gyro_state.a2 = (1 - K/Q + K^2) / norm;
            end
            
            % 各軸にBiquadフィルタ適用
            w_filt = zeros(3,1);
            for i = 1:3
                % Direct Form II
                w = w_meas(i) - obj.gyro_state.a1 * obj.gyro_state.y1(i) ...
                              - obj.gyro_state.a2 * obj.gyro_state.y2(i);
                
                w_filt(i) = obj.gyro_state.b0 * w ...
                          + obj.gyro_state.b1 * obj.gyro_state.x1(i) ...
                          + obj.gyro_state.b2 * obj.gyro_state.x2(i);
                
                % 状態更新
                obj.gyro_state.x2(i) = obj.gyro_state.x1(i);
                obj.gyro_state.x1(i) = w;
                obj.gyro_state.y2(i) = obj.gyro_state.y1(i);
                obj.gyro_state.y1(i) = w_filt(i);
            end
        end
        
        %% 磁気計フィルタ
        function [m_filt, is_outlier] = filter_mag(obj, m_meas)
            % FILTER_MAG 磁気計EMAフィルタ + 外れ値検出
            %
            % 入力:
            %   m_meas - 計測磁場 [3x1] (nT or 正規化)
            %
            % 出力:
            %   m_filt     - フィルタ済み磁場 [3x1]
            %   is_outlier - 外れ値判定フラグ
            
            % 前回値との差分
            if norm(obj.mag_state.filtered) < 1e-6
                % 初回
                obj.mag_state.filtered = m_meas;
                m_filt = m_meas;
                is_outlier = false;
                return;
            end
            
            diff = m_meas - obj.mag_state.filtered;
            diff_norm = norm(diff);
            
            % ノイズ推定
            if isempty(obj.mag_state.history)
                noise_std = diff_norm;
            else
                noise_std = std(obj.mag_state.history);
            end
            
            % 外れ値判定 (5σ基準 - 磁場は環境変動が大きい)
            threshold = 5.0 * max(noise_std, 5.0);
            is_outlier = (diff_norm > threshold);
            
            if is_outlier
                m_filt = obj.mag_state.filtered;
            else
                % EMAフィルタ
                alpha = obj.mag_state.alpha;
                m_filt = alpha * m_meas + (1 - alpha) * obj.mag_state.filtered;
                obj.mag_state.filtered = m_filt;
                
                % 履歴更新
                obj.mag_state.history = [obj.mag_state.history; diff_norm];
                if length(obj.mag_state.history) > 20
                    obj.mag_state.history = obj.mag_state.history(2:end);
                end
            end
        end
        
        %% GPSフィルタ
        function [p_filt, v_filt] = filter_gps(obj, p_meas, dt)
            % FILTER_GPS GPSアルファベータフィルタ
            %
            % 入力:
            %   p_meas - GPS位置計測 [3x1] (m)
            %   dt     - サンプリング時間 (秒)
            %
            % 出力:
            %   p_filt - フィルタ済み位置 [3x1]
            %   v_filt - 推定速度 [3x1]
            
            % 予測
            p_pred = obj.gps_state.filtered + obj.gps_state.vel * dt;
            
            % イノベーション
            innov = p_meas - p_pred;
            
            % 更新
            alpha = obj.gps_state.alpha;
            beta = obj.gps_state.beta;
            
            p_filt = p_pred + alpha * innov;
            v_filt = obj.gps_state.vel + (beta / dt) * innov;
            
            % 状態保存
            obj.gps_state.filtered = p_filt;
            obj.gps_state.vel = v_filt;
        end
        
        %% 気圧計フィルタ
        function h_filt = filter_baro(obj, h_meas)
            % FILTER_BARO 気圧計EMAフィルタ
            %
            % 入力:
            %   h_meas - 高度計測 (m)
            %
            % 出力:
            %   h_filt - フィルタ済み高度
            
            % EMAフィルタ
            alpha = obj.baro_state.alpha;
            h_filt = alpha * h_meas + (1 - alpha) * obj.baro_state.filtered;
            obj.baro_state.filtered = h_filt;
        end
        
        %% リセット
        function reset(obj)
            % すべてのフィルタ状態をリセット
            obj.accel_state.filtered = zeros(3,1);
            obj.accel_state.history = [];
            obj.gyro_state.x1 = zeros(3,1);
            obj.gyro_state.x2 = zeros(3,1);
            obj.gyro_state.y1 = zeros(3,1);
            obj.gyro_state.y2 = zeros(3,1);
            obj.mag_state.filtered = zeros(3,1);
            obj.mag_state.history = [];
            obj.gps_state.filtered = zeros(3,1);
            obj.gps_state.vel = zeros(3,1);
            obj.baro_state.filtered = 0;
        end
    end
end
