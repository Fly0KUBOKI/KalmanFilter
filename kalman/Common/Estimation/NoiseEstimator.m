classdef NoiseEstimator < handle
    % NoiseEstimator 簡易ノイズ推定器
    
    properties
        R_accel
        R_gyro
        R_mag
        R_baro
        R_gps
    end
    
    methods
        function obj = NoiseEstimator(~)
            obj.R_accel = ones(3,1) * 0.01;
            obj.R_gyro = ones(3,1) * 0.001;
            obj.R_mag = ones(3,1) * 1.0;
            obj.R_baro = 0.1;
            obj.R_gps = ones(3,1) * 1.0;
        end
        
        function R = getRnoise(obj, sensor_type)
            switch sensor_type
                case 'accel'
                    R = diag(obj.R_accel);
                case 'gyro'
                    R = diag(obj.R_gyro);
                case 'mag'
                    R = diag(obj.R_mag);
                case 'baro'
                    R = obj.R_baro;
                case 'gps'
                    R = diag(obj.R_gps);
                otherwise
                    R = eye(3);
            end
        end
        
        function [thr_vec, scalar_thr] = getThreshold(obj, sensor_type, sigma_mult)
            if nargin < 3
                sigma_mult = 2.0;
            end
            
            switch sensor_type
                case 'accel'
                    thr_vec = sqrt(obj.R_accel) * sigma_mult;
                    scalar_thr = mean(thr_vec);
                case 'gyro'
                    thr_vec = sqrt(obj.R_gyro) * sigma_mult;
                    scalar_thr = mean(thr_vec);
                otherwise
                    thr_vec = ones(3,1) * 0.1;
                    scalar_thr = 0.1;
            end
        end
        
        function estimate(obj, sensor_type, ~, ~, ~)
            % 簡易版：何もしない（推定値を固定）
        end
    end
end
