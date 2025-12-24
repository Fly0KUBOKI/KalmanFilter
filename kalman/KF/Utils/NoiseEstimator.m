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
                if exist('mex_kf_utils','file')==2
                    mex_kf_utils('sensor','reset');
                    % NOTE: Do NOT auto-sync initial R from MEX here.
                    % Previous behavior called obj.sync_from_mex() which allowed
                    % compiled MEX to overwrite MATLAB-side NoiseEstimator
                    % defaults. To keep MATLAB estimator authoritative and
                    % avoid unit/scale mismatches, skip automatic sync.
                    % If needed, call obj.sync_from_mex(sensor_type) manually.
                end
            catch
            end
        end

        function estimate(obj, sensor_type, innovation, H, P)
            % Update noise estimates locally in MATLAB to avoid implicit
            % overwriting from compiled MEX and to keep units consistent.
            % innovation: nx1, H: nxm, P: mxm
            if nargin < 4
                error('NoiseEstimator:InvalidArgs','estimate requires sensor_type, innovation, H, P');
            end

            % compute H * P * H'
            HP = H * P;
            HPHT = HP * H';

            innov_len = size(innovation,1);
            innov_var = zeros(innov_len,1);
            for i=1:innov_len
                innov_sq = double(innovation(i))^2;
                hpht_ii = double(HPHT(i,i));
                innov_var(i) = max(innov_sq - hpht_ii, eps);
            end

            % capture R before update for debug
            switch sensor_type
                case 'accel', R_before = obj.R_accel;
                case 'gyro',  R_before = obj.R_gyro;
                case 'mag',   R_before = obj.R_mag;
                case 'gps',   R_before = obj.R_gps;
                case 'baro',  R_before = obj.R_baro;
                otherwise, R_before = [];
            end

            switch sensor_type
                case 'accel'
                    obj = obj.update_noise_matlab('accel', innov_var);
                case 'gyro'
                    obj = obj.update_noise_matlab('gyro', innov_var);
                case 'mag'
                    obj = obj.update_noise_matlab('mag', innov_var);
                case 'gps'
                    obj = obj.update_noise_matlab('gps', innov_var);
                case 'baro'
                    % baro is scalar
                    var = innov_var(1);
                    obj.count_baro = obj.count_baro + 1;
                    if obj.count_baro <= obj.warmup_samples
                        obj.sum_baro = obj.sum_baro + var;
                        obj.R_baro = obj.sum_baro / obj.count_baro;
                    else
                        max_allowed = obj.R_baro * obj.OUTLIER_FACTOR;
                        var = min(var, max_allowed);
                        obj.R_baro = (1 - obj.alpha) * obj.R_baro + obj.alpha * var;
                    end
                    obj.R_baro = min(max(obj.R_baro, obj.R_ABS_MIN), obj.R_ABS_MAX);
                otherwise
                    error('Unknown sensor type: %s', sensor_type);
            end

            % capture R after update for debug
            switch sensor_type
                case 'accel', R_after = obj.R_accel;
                case 'gyro',  R_after = obj.R_gyro;
                case 'mag',   R_after = obj.R_mag;
                case 'gps',   R_after = obj.R_gps;
                case 'baro',  R_after = obj.R_baro;
                otherwise, R_after = [];
            end

            % Write debug JSON if enabled: drop a trigger file in Results/
            try
                repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
                results_dir = fullfile(repo_root,'Results');
                trigger = fullfile(results_dir,'noise_estimator_debug_enable.txt');
                if exist(trigger,'file')==2
                    dbg.sensor = sensor_type;
                    dbg.timestamp = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS'));
                    dbg.innovation = double(innovation(:));
                    dbg.HPHT = double(HPHT);
                    dbg.innov_var = double(innov_var(:));
                    dbg.R_before = double(R_before);
                    dbg.R_after = double(R_after);
                    dbg.alpha = double(obj.alpha);
                    dbg.counts = struct('accel',obj.count_accel,'gyro',obj.count_gyro,'mag',obj.count_mag,'gps',obj.count_gps,'baro',obj.count_baro);
                    fname = sprintf('noise_estimate_debug_%s.json',datestr(now,'yyyymmdd_HHMMSS_FFF'));
                    fpath = fullfile(results_dir,fname);
                    jsontxt = jsonencode(dbg);
                    % make JSON pretty-ish (insert newlines)
                    fid = fopen(fpath,'w');
                    if fid ~= -1
                        fwrite(fid, jsontxt, 'char'); fclose(fid);
                    end
                end
            catch
                % ignore debug write failures
            end
        end

        function obj = update_noise_matlab(obj, sensor_type, innov_var)
            % Helper: update vector-valued noise (3x1) using EMA like C++ implementation
            switch sensor_type
                case 'accel'
                    Rv = obj.R_accel; cnt = obj.count_accel; sm = obj.sum_accel;
                case 'gyro'
                    Rv = obj.R_gyro; cnt = obj.count_gyro; sm = obj.sum_gyro;
                case 'mag'
                    Rv = obj.R_mag; cnt = obj.count_mag; sm = obj.sum_mag;
                case 'gps'
                    Rv = obj.R_gps; cnt = obj.count_gps; sm = obj.sum_gps;
                otherwise
                    return;
            end

            cnt = cnt + 1;
            if cnt <= obj.warmup_samples
                sm = sm + innov_var;
                Rv = sm / cnt;
            else
                for i=1:numel(innov_var)
                    max_allowed = Rv(i) * obj.OUTLIER_FACTOR;
                    v = min(innov_var(i), max_allowed);
                    Rv(i) = (1 - obj.alpha) * Rv(i) + obj.alpha * v;
                end
            end

            for i=1:numel(Rv)
                Rv(i) = min(max(Rv(i), obj.R_ABS_MIN), obj.R_ABS_MAX);
            end

            switch sensor_type
                case 'accel'
                    obj.R_accel = Rv; obj.count_accel = cnt; obj.sum_accel = sm;
                case 'gyro'
                    obj.R_gyro = Rv; obj.count_gyro = cnt; obj.sum_gyro = sm;
                case 'mag'
                    obj.R_mag = Rv; obj.count_mag = cnt; obj.sum_mag = sm;
                case 'gps'
                    obj.R_gps = Rv; obj.count_gps = cnt; obj.sum_gps = sm;
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
                    % Return GPS noise as a 3x1 variance vector (preferred for MEX params)
                    % Many callers wrap with diag(...) if they need a matrix. Returning
                    % a column vector avoids ambiguity when code does tmpR(:) or
                    % sets mex_params.noise_gps = tmpR(:).
                    R = obj.R_gps(:);
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
            % Pull updated R from compiled sensors via mex_kf_utils for one sensor or all
            if nargin < 2
                sensors = {'accel','gyro','mag','gps','baro'};
            else
                sensors = {sensor_type};
            end
            for i=1:numel(sensors)
                s = sensors{i};
                try
                    Rm = mex_kf_utils('sensor', 'get_R', s);
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
