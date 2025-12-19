classdef NoiseEstimator < handle
    % Compatibility shim that exposes the original MATLAB API but delegates
    % computation to the MEX implementation when available.
    properties
        % Noise variances (per-axis for vector sensors)
        R_accel     % 3x1
        R_gyro      % 3x1
        R_mag       % 3x1
        R_gps       % 3x1
        R_baro      % scalar

        % Internal counters / sums (kept for compatibility; not used by MEX)
        count_accel
        count_gyro
        count_mag
        count_gps
        count_baro

        sum_accel
        sum_gyro
        sum_mag
        sum_gps
        sum_baro

        alpha
        alpha_count
        warmup_samples
    end

    properties (Constant)
        R_ABS_MAX = 1e6;
        R_ABS_MIN = eps;
        OUTLIER_FACTOR = 20;
    end

    methods
        function obj = NoiseEstimator(warmup_samples)
            if nargin < 1
                warmup_samples = 10;
            end
            obj.warmup_samples = warmup_samples;

            % initialize counters/sums
            obj.count_accel = 0; obj.count_gyro = 0; obj.count_mag = 0; obj.count_gps = 0; obj.count_baro = 0;
            obj.sum_accel = zeros(3,1); obj.sum_gyro = zeros(3,1); obj.sum_mag = zeros(3,1); obj.sum_gps = zeros(3,1); obj.sum_baro = 0;

            % default noise values (match original MATLAB defaults)
            obj.R_accel = ones(3,1) * 0.01;
            obj.R_gyro  = ones(3,1) * (deg2rad(0.1)^2);
            obj.R_mag   = ones(3,1) * 5.0^2;
            obj.R_gps   = [3^2; 3^2; 5^2];
            obj.R_baro  = 1.0^2;

            obj.alpha = 0.01;
            obj.alpha_count = 0;

            % Try to add compiled MEX bin to path if present and reset
            try
                p = fileparts(fileparts(fileparts(mfilename('fullpath'))));
                mexbin = fullfile(p, 'cpp', 'bin');
                if exist(mexbin,'dir')==7
                    addpath(mexbin);
                end
            catch
            end
            try
                if exist('mex_sensor_filter','file')==3 || exist('mex_sensor_filter','file')==2
                    mex_sensor_filter('reset');
                    % Sync initial R values from MEX implementation
                    obj.sync_from_mex();
                end
            catch
            end
        end

        function estimate(obj, sensor_type, innovation, H, P)
            % Update via MEX (preferred). After calling MEX, sync local properties.
            if exist('mex_sensor_filter','file')
                mex_sensor_filter('noise_estimate', sensor_type, innovation, H, P);
                obj.sync_from_mex(sensor_type);
            else
                error('NoiseEstimator:MissingMEX','mex_sensor_filter not found on path');
            end
        end

        function R = getRnoise(obj, sensor_type)
            % Return covariance matrix consistent with original API
            switch sensor_type
                case 'accel'
                    R = diag(obj.R_accel);
                case 'gyro'
                    R = diag(obj.R_gyro);
                case 'mag'
                    mag_min_var = 1e-2;
                    R = diag(max(obj.R_mag, mag_min_var));
                case 'gps'
                    R = diag(obj.R_gps);
                case 'baro'
                    R = obj.R_baro;
                otherwise
                    error('Unknown sensor type: %s', sensor_type);
            end
        end

        function [vec, scalar] = getThreshold(obj, sensor_type, sigma_multiplier)
            if nargin < 3, sigma_multiplier = 2.0; end
            switch sensor_type
                case 'accel', Rvar = obj.R_accel;
                case 'gyro',  Rvar = obj.R_gyro;
                case 'mag',   Rvar = obj.R_mag;
                case 'gps',   Rvar = obj.R_gps;
                case 'baro',  Rvar = obj.R_baro;
                otherwise, error('Unknown sensor type: %s', sensor_type);
            end
            std_vec = sqrt(max(Rvar, eps));
            vec = sigma_multiplier * std_vec;
            scalar = mean(vec);
        end
    end

    methods (Access = private)
        function sync_from_mex(obj, sensor_type)
            % Pull updated R from mex_sensor_filter for one sensor or all
            if nargin < 2
                sensors = {'accel','gyro','mag','gps','baro'};
            else
                sensors = {sensor_type};
            end
            for i=1:numel(sensors)
                s = sensors{i};
                try
                    Rm = mex_sensor_filter('get_R', s);
                    if strcmp(s,'baro')
                        obj.R_baro = double(Rm);
                    else
                        v = diag(Rm);
                        switch s
                            case 'accel', obj.R_accel = v(:);
                            case 'gyro',  obj.R_gyro  = v(:);
                            case 'mag',   obj.R_mag   = v(:);
                            case 'gps',   obj.R_gps   = v(:);
                        end
                    end
                catch
                    % ignore if MEX not present or returned unexpected format
                end
            end
        end
    end
end
