classdef MathUtils
    % MATHUTILS 数学ユーティリティ関数集
    % C++移行を想定した設計
    
    properties (Constant)
        EPS = 1.0e-9;
    end
    
    methods (Static)
        %% 角度処理
        function angle_wrapped = wrap_to_pi(angle)
            % 角度を[-π, π]に正規化 (rad)
            angle_wrapped = mod(angle + pi, 2*pi) - pi;
        end
        
        function angle_wrapped = wrap_to_180(angle)
            % 角度を[-180, 180]に正規化 (度)
            angle_wrapped = mod(angle + 180, 360) - 180;
        end
        
        function angle_diff = angle_difference(a1, a2)
            % 角度差を[-π, π]で計算 (rad)
            diff = a2 - a1;
            angle_diff = MathUtils.wrap_to_pi(diff);
        end
        
        %% ベクトル・行列操作
        function v_normalized = normalize_vector(v)
            % ベクトル正規化
            v = v(:);
            n = norm(v);
            if n < MathUtils.EPS
                v_normalized = zeros(size(v));
            else
                v_normalized = v / n;
            end
        end
        
        function [v_clipped, clipped] = clip_vector(v, max_norm)
            % ベクトルのノルムを制限
            v = v(:);
            n = norm(v);
            if n > max_norm
                v_clipped = v * (max_norm / n);
                clipped = true;
            else
                v_clipped = v;
                clipped = false;
            end
        end
        
        function M_sym = enforce_symmetry(M)
            % 行列の対称性を強制
            M_sym = 0.5 * (M + M');
        end
        
        %% 数値安定化
        function x_safe = safe_divide(numerator, denominator, default_value)
            % 安全な除算（ゼロ除算回避）
            if nargin < 3
                default_value = 0;
            end
            
            if abs(denominator) < MathUtils.EPS
                x_safe = default_value;
            else
                x_safe = numerator / denominator;
            end
        end
        
        function x_safe = safe_sqrt(x)
            % 安全な平方根（負数回避）
            x_safe = sqrt(max(x, 0));
        end
        
        function x_safe = safe_asin(x)
            % 安全なasin（定義域制限）
            x_clamped = max(min(x, 1.0), -1.0);
            x_safe = asin(x_clamped);
        end
        
        function x_safe = safe_acos(x)
            % 安全なacos（定義域制限）
            x_clamped = max(min(x, 1.0), -1.0);
            x_safe = acos(x_clamped);
        end
        
        %% 統計
        function [mean_val, std_val] = robust_statistics(data, outlier_threshold)
            % ロバストな平均・標準偏差計算（外れ値除外）
            if nargin < 2
                outlier_threshold = 3.0;  % 3σ
            end
            
            data = data(:);
            
            % 初期推定
            mu = median(data);
            sigma = 1.4826 * mad(data, 1);  % MAD
            
            % 外れ値除外
            z_score = abs(data - mu) / (sigma + MathUtils.EPS);
            inliers = z_score < outlier_threshold;
            
            if sum(inliers) > 0
                mean_val = mean(data(inliers));
                std_val = std(data(inliers));
            else
                mean_val = mu;
                std_val = sigma;
            end
        end
        
        %% 行列分解
        function [L, success] = safe_cholesky(A)
            % 安全なCholesky分解
            % 正定値でない場合は正則化
            success = true;
            
            try
                L = chol(A, 'lower');
            catch
                % 正則化して再試行
                min_eig = min(eig(A));
                if min_eig <= 0
                    epsilon = abs(min_eig) + 1e-6;
                    A = A + epsilon * eye(size(A));
                end
                
                try
                    L = chol(A, 'lower');
                catch
                    % それでも失敗したら単位行列
                    L = eye(size(A));
                    success = false;
                end
            end
        end
        
        %% 座標変換
        function [x_enu, y_enu, z_enu] = lla_to_enu(lat, lon, alt, lat0, lon0, alt0)
            % 緯度経度高度からENU座標への変換
            % 簡易版（球面近似）
            
            % 度からメートルへの変換係数
            deg_to_m_lat = 1.0 / 9.0e-6;
            deg_to_m_lon = 1.0 / (9.0e-6 / cosd(lat0));
            
            % ENU座標
            x_enu = (lon - lon0) * deg_to_m_lon;  % East
            y_enu = (lat - lat0) * deg_to_m_lat;  % North
            z_enu = alt - alt0;                    % Up
        end
        
        function [lat, lon, alt] = enu_to_lla(x_enu, y_enu, z_enu, lat0, lon0, alt0)
            % ENU座標から緯度経度高度への変換
            % 簡易版（球面近似）
            
            % メートルから度への変換係数
            m_to_deg_lat = 9.0e-6;
            m_to_deg_lon = 9.0e-6 / cosd(lat0);
            
            lon = lon0 + x_enu * m_to_deg_lon;
            lat = lat0 + y_enu * m_to_deg_lat;
            alt = alt0 + z_enu;
        end
        
        %% 補間
        function y = linear_interpolate(x, x1, y1, x2, y2)
            % 線形補間
            if abs(x2 - x1) < MathUtils.EPS
                y = y1;
            else
                t = (x - x1) / (x2 - x1);
                y = y1 + t * (y2 - y1);
            end
        end
    end
end
