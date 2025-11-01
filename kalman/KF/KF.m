classdef KF < handle
    % KF  基底カルマンフィルタクラス（ESKF/EKF/UKF が継承）
    % 共通プロパティとユーティリティを定義する。

    properties
        % 共通共分散・ノイズ
        P           % 状態共分散行列
        Q           % プロセスノイズ共分散

        % ノイズ推定器
        noiseEstimator

        % サンプリング / 周期
        dt
        freq_mag
        freq_gps
        freq_baro

        % 参照値
        gps_origin
        g

        % その他
        gyro_noise_threshold
    end

    methods
        function obj = KF()
            % 基底コンストラクタ：最小限のデフォルトを設定
            obj.P = [];
            obj.Q = [];
            obj.noiseEstimator = [];
            obj.dt = [];
            obj.freq_mag = 0;
            obj.freq_gps = 0;
            obj.freq_baro = 0;
            obj.gps_origin = [];
            obj.g = [0;0;9.81];
            obj.gyro_noise_threshold = 0;
        end

        function [roll,pitch,yaw] = getEulerFromQuat(~, q)
            % GETEULERFROMQUAT  与えられたクォータニオンからオイラー角を返すヘルパ
            e = quat_lib('quat_to_euler', q);
            roll = e(2); pitch = e(1); yaw = e(3);
        end
    end
end
