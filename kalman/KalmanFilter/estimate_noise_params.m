function [Q, R_gps, R_mag, R_baro] = estimate_noise_params(obs)
    % ESTIMATE_NOISE_PARAMS  観測データから簡易ノイズ推定を行う
    % 入力 obs は read_csv の出力構造体

    accel_norm = sqrt(obs.ax.^2 + obs.ay.^2 + obs.az.^2);
    is_stationary = abs(accel_norm - 9.81) < 0.5;

    if any(is_stationary)
        sigma_a = std(accel_norm(is_stationary));
        gyro_norm = sqrt(obs.wx.^2 + obs.wy.^2 + obs.wz.^2);
        sigma_g = std(gyro_norm(is_stationary));
        % sigma_g = deg2rad(sigma_g);
        mag_norm = sqrt(obs.mx.^2 + obs.my.^2 + obs.mz.^2);
        sigma_m = std(mag_norm(is_stationary));
    else
        % デフォルト値
        sigma_a = 0.1;
        sigma_g = 0.01;
        sigma_m = 0.5;
    end

    % プロセスノイズ Q (15x15)
    Q = zeros(15);
    Q(4:6,4:6) = eye(3) * (0.1^2);
    Q(7:9,7:9) = eye(3) * (0.01^2);
    Q(10:12,10:12) = eye(3) * (sigma_a^2 * 1e-4);
    Q(13:15,13:15) = eye(3) * (sigma_g^2 * 1e-5);

    % 観測ノイズ
    R_gps = diag([3^2, 3^2, 5^2]);
    R_mag = eye(3) * (sigma_m^2);
    R_baro = 1.0^2;
end
