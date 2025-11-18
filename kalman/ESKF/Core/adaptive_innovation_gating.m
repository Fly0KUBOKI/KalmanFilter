function [should_accept, innov_scaled, gain_scale] = adaptive_innovation_gating(innov, S, sensor_type)
% ADAPTIVE_INNOVATION_GATING  ArduPilot風の適応的イノベーションゲーティング
%
% ArduPilot NavEKF2/3のイノベーション管理手法を実装
% - Chi-square検定によるイノベーションの妥当性チェック
% - 適応的なゲインスケーリング
% - センサータイプ別の閾値管理
%
% 入力:
%   innov       - イノベーション (nx1)
%   S           - イノベーション共分散 (nxn)
%   sensor_type - センサータイプ ('accel', 'mag', 'gps', 'baro')
%
% 出力:
%   should_accept - 更新を受け入れるか (true/false)
%   innov_scaled  - スケーリングされたイノベーション
%   gain_scale    - ゲインスケーリング係数

% ArduPilotのイノベーション制限（5-sigma）
MAX_INNOV_SIGMA = 5.0;

% センサータイプ別のChi-square閾値
% 自由度に応じた99%信頼区間の閾値
switch sensor_type
    case 'accel'
        chi2_threshold = 11.345;  % 3自由度, 99%信頼区間
        max_innov_norm = 2.0;     % m/s^2
    case 'mag'
        chi2_threshold = 11.345;  % 3自由度
        max_innov_norm = 0.5;     % Tesla
    case 'gps'
        chi2_threshold = 11.345;  % 3自由度
        max_innov_norm = 10.0;    % m
    case 'baro'
        chi2_threshold = 6.635;   % 1自由度
        max_innov_norm = 5.0;     % m
    otherwise
        chi2_threshold = 11.345;
        max_innov_norm = 1.0;
end

% Chi-square検定
% マハラノビス距離の二乗: d^2 = y' * S^-1 * y
try
    S_inv = inv(S + eye(size(S)) * 1e-12);  % 数値安定性
    mahal_dist_sq = innov' * S_inv * innov;
catch
    % 特異行列の場合は保守的に拒否
    should_accept = false;
    innov_scaled = innov;
    gain_scale = 0.0;
    return;
end

% イノベーションのノルムチェック
innov_norm = norm(innov);

% ArduPilot風の適応的ゲーティング
if mahal_dist_sq > chi2_threshold || innov_norm > max_innov_norm
    % イノベーションが大きすぎる場合
    
    % 5-sigma制限（ArduPilot NavEKF2のcompression scale factor）
    if mahal_dist_sq > MAX_INNOV_SIGMA^2
        % イノベーションを圧縮
        compression_factor = MAX_INNOV_SIGMA / sqrt(mahal_dist_sq);
        innov_scaled = innov * compression_factor;
        gain_scale = compression_factor;
        should_accept = true;  % 圧縮して受け入れ
    else
        % 適度に大きい場合は減衰して受け入れ
        attenuation = chi2_threshold / mahal_dist_sq;
        innov_scaled = innov * sqrt(attenuation);
        gain_scale = sqrt(attenuation);
        should_accept = true;
    end
else
    % イノベーションが妥当な範囲
    innov_scaled = innov;
    gain_scale = 1.0;
    should_accept = true;
end

end
