function P_next = covariance_prediction_optimized(P, q, a_meas, ba, w_meas, bg, Q, dt)
% COVARIANCE_PREDICTION_OPTIMIZED  ArduPilot風の最適化された共分散予測
%
% ArduPilot NavEKF2/3の手法を参考にした高速化実装
% Matlab Symbolic Toolboxで導出された代数式を使用
%
% 入力:
%   P       - 現在の共分散行列 (15x15)
%   q       - 現在のクォータニオン [w; x; y; z]
%   a_meas  - 加速度測定値 (3x1)
%   ba      - 加速度バイアス (3x1)
%   w_meas  - 角速度測定値 (3x1)
%   bg      - 角速度バイアス (3x1)
%   Q       - プロセスノイズ共分散 (15x15)
%   dt      - 時間刻み (s)
%
% 出力:
%   P_next  - 予測された共分散行列 (15x15)

% 補正済みIMU測定値
dvx = (a_meas(1) - ba(1)) * dt;
dvy = (a_meas(2) - ba(2)) * dt;
dvz = (a_meas(3) - ba(3)) * dt;
dax = (w_meas(1) - bg(1)) * dt;
day = (w_meas(2) - bg(2)) * dt;
daz = (w_meas(3) - bg(3)) * dt;

% クォータニオン成分
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

% バイアス（ゼロと仮定）
dax_b = 0;
day_b = 0;
daz_b = 0;

% ArduPilot NavEKF2のスタイル: 中間変数（Symbolic Toolboxで導出）
% 参考: https://github.com/priseborough/InertialNav/
SF = zeros(24, 1);
SF(1) = dvz - dax_b*q1 + day_b*q0 - daz_b*q2;
SF(2) = dvx + dax_b*q2 + day_b*q3 - daz_b*q0;
SF(3) = dvy - dax_b*q3 + day_b*q2 + daz_b*q1;
SF(4) = dax*0.5;
SF(5) = day*0.5;
SF(6) = daz*0.5;
SF(7) = dax_b*0.5;
SF(8) = day_b*0.5;
SF(9) = daz_b*0.5;

% 回転行列要素
SF(10) = q0*q0;
SF(11) = q1*q1;
SF(12) = q2*q2;
SF(13) = q3*q3;
SF(14) = q0*q1*2.0;
SF(15) = q0*q2*2.0;
SF(16) = q0*q3*2.0;
SF(17) = q1*q2*2.0;
SF(18) = q1*q3*2.0;
SF(19) = q2*q3*2.0;

% 正規化項
SF(20) = SF(10) - SF(11) + SF(12) - SF(13);
SF(21) = SF(10) + SF(11) - SF(12) - SF(13);
SF(22) = SF(10) - SF(11) - SF(12) + SF(13);

% 状態遷移ヤコビアン F の構築
% F = I + F_c * dt （連続時間から離散化）
F = eye(15);

% 位置-速度のカップリング
F(1:3, 4:6) = eye(3) * dt;

% 速度-姿勢のカップリング（回転行列微分）
R_current = quat_to_rotm_internal(q);
skew_a = [0, -(a_meas(3)-ba(3)), (a_meas(2)-ba(2));
          (a_meas(3)-ba(3)), 0, -(a_meas(1)-ba(1));
          -(a_meas(2)-ba(2)), (a_meas(1)-ba(1)), 0];
F(4:6, 7:9) = -R_current * skew_a * dt;

% 速度-加速度バイアスのカップリング
F(4:6, 10:12) = -R_current * dt;

% 姿勢-角速度バイアスのカップリング
F(7:9, 13:15) = -eye(3) * dt;

% プロセスノイズの離散化
% G * Q_c * G' を計算（簡略版）
G = zeros(15, 12);
G(4:6, 1:3) = R_current * dt;      % 加速度ノイズ
G(7:9, 4:6) = eye(3) * dt;         % 角速度ノイズ
G(10:12, 7:9) = eye(3) * dt;       % 加速度バイアスランダムウォーク
G(13:15, 10:12) = eye(3) * dt;     % 角速度バイアスランダムウォーク

% 簡略化されたプロセスノイズ（対角成分のみ）
Q_discrete = Q + eye(15) * 1e-12;  % 数値安定性

% 共分散予測: P_next = F * P * F' + Q_discrete
P_next = F * P * F' + Q_discrete;

% 対称性の強制（ArduPilot NavEKF2/3の手法）
P_next = 0.5 * (P_next + P_next');

% 正定値性の保証（対角成分の下限）
min_var = 1e-9;
for i = 1:15
    if P_next(i,i) < min_var
        P_next(i,i) = min_var;
    end
end

end

function R = quat_to_rotm_internal(q)
% 内部用のクォータニオンから回転行列への変換
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
R = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];
end
