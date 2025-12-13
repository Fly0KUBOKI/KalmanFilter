function output = call_unified_filter(obj, input_struct)
% CALL_UNIFIED_FILTER  C++統合フィルタを呼び出す
%
% Usage:
%   output = obj.call_unified_filter(input_struct)
%
% Input:
%   input_struct - FilterInput構造体
%       .dt             : サンプリング時間 [s]
%       .accel          : 加速度 [m/s^2] (3x1)
%       .gyro           : 角速度 [rad/s] (3x1)
%       .mag            : 磁場 (3x1)
%       .mag_valid      : 磁気計有効フラグ
%       .gps_pos        : GPS位置 [m] (3x1, NED)
%       .gps_valid      : GPS有効フラグ
%       .baro_alt       : 気圧高度 [m]
%       .baro_valid     : 気圧計有効フラグ
%       .prev_mag       : 前回磁場値 (3x1)
%       .prev_gps_pos   : 前回GPS位置 (3x1)
%       .prev_baro_alt  : 前回気圧高度
%       .g              : 重力ベクトル (3x1, NED)
%       .mag_ref        : 磁場基準 (3x1, NED)
%       .noise_*        : 各センサーのノイズ標準偏差
%       .alpha, beta, kappa : UKFパラメータ
%
% Output:
%   output - FilterOutput構造体
%       .position       : 位置 [m] (3x1, NED)
%       .velocity       : 速度 [m/s] (3x1, NED)
%       .quaternion     : 姿勢クォータニオン [qw, qx, qy, qz] (4x1)
%       .accel_bias     : 加速度バイアス (3x1)
%       .gyro_bias      : ジャイロバイアス (3x1)
%       .covariance     : 共分散行列 (15x15)
%       .roll, pitch, yaw : オイラー角 [deg]
%       .innovation_norm_* : 各センサーのイノベーションノルム
%       .divergence_detected : 発散検知フラグ
%       .reset_occurred : リセット発生フラグ

% 現在の状態を構造体にパック
prev_state = struct();
prev_state.p = obj.p(:);
prev_state.v = obj.v(:);
prev_state.q = obj.q(:);
prev_state.ba = obj.ba(:);
prev_state.bg = obj.bg(:);
prev_state.P = obj.P;

% MEX呼び出し
output = mex_unified_filter(prev_state, input_struct);

% ESKFの状態を更新
obj.p = output.position(:);
obj.v = output.velocity(:);
obj.q = output.quaternion(:);
obj.ba = output.accel_bias(:);
obj.bg = output.gyro_bias(:);
obj.P = output.covariance;

end
