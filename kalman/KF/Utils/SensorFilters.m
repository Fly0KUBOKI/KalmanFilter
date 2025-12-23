classdef SensorFilters
% SensorFilters - Unified MATLAB wrapper that delegates sensor filter
% operations to the mex implementation when available.
%
% Usage examples:
%  SensorFilters.reset();
%  SensorFilters.accel_config([0.3,10,3.0,0.1]);
%  [a_filt, is_out] = SensorFilters.accel(a_meas, a_expected);
%  R = SensorFilters.get_R('accel');

    methods(Static)
        function reset()
            mex_sensor_filter('reset');
        end

        function reset_zero()
            mex_sensor_filter('reset_zero');
        end

        function accel_config(cfg)
            mex_sensor_filter('accel_config', cfg);
        end

        function [a_filt, is_out] = accel(a_meas, a_expected)
            % Prefer MEX but validate outputs; fall back to simple MATLAB pass-through
            is_out = false;
            if exist('mex_sensor_filter','file') == 3
                try
                    if nargout <= 1
                        a_filt = mex_sensor_filter('accel', a_meas, a_expected);
                    else
                        [a_filt, is_out] = mex_sensor_filter('accel', a_meas, a_expected);
                    end
                    if ~SensorFilters.validate_sensor_output('accel', a_filt)
                        try
                            mex_sensor_filter('reset');
                        catch
                        end
                        if nargout <= 1
                            a_filt = mex_sensor_filter('accel', a_meas, a_expected);
                        else
                            [a_filt, is_out] = mex_sensor_filter('accel', a_meas, a_expected);
                        end
                        if ~SensorFilters.validate_sensor_output('accel', a_filt)
                            warning('SensorFilters:AccelMEXInvalid','Accel MEX produced invalid output; using MATLAB fallback.');
                            a_filt = SensorFilters.accel_matlab_fallback(a_meas);
                            is_out = false;
                        end
                    end
                catch ME
                    warning('SensorFilters:AccelMEXFail','Accel MEX failed (%s). Using MATLAB fallback.', ME.message);
                    a_filt = SensorFilters.accel_matlab_fallback(a_meas);
                    is_out = false;
                end
            else
                a_filt = SensorFilters.accel_matlab_fallback(a_meas);
                is_out = false;
            end
        end

        function [m_filt, is_out] = mag(m_meas, m_expected)
            % Defensive normalization: ensure inputs are 3-element column vectors
            validateattributes(m_meas, {'numeric'}, {'nonempty'}, mfilename, 'm_meas');
            if numel(m_meas) == 3
                m_meas = m_meas(:);
            elseif ismatrix(m_meas) && any(size(m_meas) == 3)
                if size(m_meas,1) == 3
                    m_meas = m_meas(:,1);
                elseif size(m_meas,2) == 3
                    m_meas = m_meas(1,:)';
                else
                    error('SensorFilters.mag: mag vectors must be length 3');
                end
            else
                error('SensorFilters.mag: mag vectors must be length 3');
            end

            if nargin < 2 || isempty(m_expected)
                m_expected = zeros(3,1);
            else
                validateattributes(m_expected, {'numeric'}, {'nonempty'}, mfilename, 'm_expected');
                if numel(m_expected) == 3
                    m_expected = m_expected(:);
                elseif ismatrix(m_expected) && any(size(m_expected) == 3)
                    if size(m_expected,1) == 3
                        m_expected = m_expected(:,1);
                    elseif size(m_expected,2) == 3
                        m_expected = m_expected(1,:)';
                    else
                        error('SensorFilters.mag: m_expected must be length 3');
                    end
                else
                    error('SensorFilters.mag: m_expected must be length 3');
                end
            end

            is_out = false;
            if exist('mex_sensor_filter','file') == 3
                try
                    if nargout <= 1
                        m_filt = mex_sensor_filter('mag', m_meas, m_expected);
                    else
                        [m_filt, is_out] = mex_sensor_filter('mag', m_meas, m_expected);
                    end
                    if ~SensorFilters.validate_sensor_output('mag', m_filt)
                        try
                            mex_sensor_filter('reset');
                        catch
                        end
                        if nargout <= 1
                            m_filt = mex_sensor_filter('mag', m_meas, m_expected);
                        else
                            [m_filt, is_out] = mex_sensor_filter('mag', m_meas, m_expected);
                        end
                        if ~SensorFilters.validate_sensor_output('mag', m_filt)
                            warning('SensorFilters:MagMEXInvalid','Mag MEX produced invalid output; using MATLAB fallback.');
                            m_filt = SensorFilters.mag_matlab_fallback(m_meas);
                            is_out = false;
                        end
                    end
                catch ME
                    warning('SensorFilters:MagMEXFail','Mag MEX failed (%s). Using MATLAB fallback.', ME.message);
                    m_filt = SensorFilters.mag_matlab_fallback(m_meas);
                    is_out = false;
                end
            else
                m_filt = SensorFilters.mag_matlab_fallback(m_meas);
                is_out = false;
            end
        end

        function [pos_out, vel_out] = gps(gps_pos, dt)
            if exist('mex_sensor_filter','file') == 3
                try
                    if nargout <= 1
                        pos_out = mex_sensor_filter('gps', gps_pos, dt);
                        vel_out = zeros(size(pos_out));
                    else
                        [pos_out, vel_out] = mex_sensor_filter('gps', gps_pos, dt);
                    end
                    if ~SensorFilters.validate_sensor_output('gps', pos_out)
                        try
                            mex_sensor_filter('reset');
                        catch
                        end
                        if nargout <= 1
                            pos_out = mex_sensor_filter('gps', gps_pos, dt);
                            vel_out = zeros(size(pos_out));
                        else
                            [pos_out, vel_out] = mex_sensor_filter('gps', gps_pos, dt);
                        end
                        if ~SensorFilters.validate_sensor_output('gps', pos_out)
                            warning('SensorFilters:GpsMEXInvalid','GPS MEX produced invalid output; using MATLAB fallback.');
                            pos_out = gps_pos;
                            vel_out = zeros(size(pos_out));
                        end
                    end
                catch ME
                    warning('SensorFilters:GpsMEXFail','GPS MEX failed (%s). Using MATLAB fallback.', ME.message);
                    pos_out = gps_pos;
                    vel_out = zeros(size(pos_out));
                end
            else
                pos_out = gps_pos;
                vel_out = zeros(size(pos_out));
            end
        end

        function [p, is_out] = baro(pressure)
            % Defensive validation for pressure input
            validateattributes(pressure, {'numeric'}, {'nonempty','finite'}, mfilename, 'pressure');
            if ~isscalar(pressure)
                if isvector(pressure) && numel(pressure) >= 1
                    % Accept a vector by taking the first element (caller likely passed row/col)
                    pressure = pressure(1);
                else
                    error('SensorFilters.baro:pressure', 'pressure must be a numeric scalar');
                end
            end
            is_out = false;
            % If mex implementation exists, prefer it but be defensive against
            % runtime failures or pathological outputs (Inf/NaN/huge values).
            if exist('mex_sensor_filter','file') == 3
                try
                    if nargout <= 1
                        p = mex_sensor_filter('baro', pressure);
                        is_out = false;
                    else
                        [p, is_out] = mex_sensor_filter('baro', pressure);
                    end
                    % Validate output
                    if ~isnumeric(p) || ~isfinite(p) || (isnumeric(p) && any(abs(p) > 1e6))
                        % Try a single reset and retry
                        try
                            mex_sensor_filter('reset');
                        catch
                        end
                        if nargout <= 1
                            p = mex_sensor_filter('baro', pressure);
                            is_out = false;
                        else
                            [p, is_out] = mex_sensor_filter('baro', pressure);
                        end
                    end
                catch ME
                    % Fall back to MATLAB implementation if MEX fails
                    warning('SensorFilters:BaroMEXFail', 'MEX baro failed (%s). Falling back to MATLAB filter.', ME.message);
                    p = SensorFilters.baro_matlab_fallback(pressure);
                    is_out = false;
                end
            else
                % No MEX available: use MATLAB fallback implementation
                p = SensorFilters.baro_matlab_fallback(pressure);
                is_out = false;
            end
        end

        function p = baro_matlab_fallback(pressure)
            % Simple EMA fallback for barometer altitude (convert pressure->altitude
            % using a rough formula and apply light exponential smoothing).
            % This fallback is intentionally simple and stable to avoid NaNs.
            % Pressure (Pa) -> approximate altitude (m) via barometric formula (ISA near sea level)
            try
                p0 = 101325; % reference
                T0 = 288.15;
                g = 9.80665;
                L = 0.0065;
                R = 287.05;
                alt = (T0 / L) * (1 - (pressure./p0).^(R*L/g));
                % EMA smoothing state is not persisted across calls; use direct value
                % to keep fallback stateless and safe.
                p = alt;
            catch
                p = 0;
            end
        end

        function R = get_R(sensor_type)
            R = mex_sensor_filter('get_R', sensor_type);
            return;
        end

        function noise_estimate(sensor_type, innov, H, P)
            mex_sensor_filter('noise_estimate', sensor_type, innov, H, P);
            return;
        end

        function [dx_out, should_skip, was_attenuated] = divergence_check(sensor_name, innov, dx_in)
            [dx_out, should_skip, was_attenuated] = mex_sensor_filter('divergence_check', sensor_name, innov, dx_in);
            return;
        end

        function Pout = divergence_regularize(P)
            Pout = mex_sensor_filter('divergence_regularize', P);
            return;
        end

        function ok = validate_sensor_output(sensor_name, out)
            % Basic checks: numeric, finite, not NaN, no extreme magnitudes
            ok = true;
            try
                if ~isnumeric(out)
                    ok = false; return;
                end
                if any(isnan(out(:))) || any(isinf(out(:)))
                    ok = false; return;
                end
                % Reject extremely large magnitudes that indicate divergence
                if any(abs(out(:)) > 1e8)
                    ok = false;
                end
            catch
                ok = false;
            end
            if ~ok
                % Log details to Results/log for post-mortem
                try
                    proj_root = fileparts(fileparts(mfilename('fullpath')));
                    log_dir = fullfile(proj_root, 'Results', 'log');
                    if ~exist(log_dir, 'dir'), mkdir(log_dir); end
                    fid = fopen(fullfile(log_dir, 'sensor_filter_errors.txt'), 'a');
                    if fid ~= -1
                        t = datestr(now);
                        fprintf(fid, '%s - Sensor %s produced invalid output:\n', t, sensor_name);
                        try
                            s = evalc('disp(out)');
                            fprintf(fid, '%s\n', s);
                        catch
                        end
                        fclose(fid);
                    end
                catch
                end
            end
        end

        function a = accel_matlab_fallback(a_meas)
            % Simple stable fallback: return measured accel (no filtering)
            a = a_meas;
        end

        function m = mag_matlab_fallback(m_meas)
            % Normalize measured mag vector as fallback
            try
                m = m_meas(:);
                n = norm(m);
                if n > 0
                    m = m / n;
                end
            catch
                m = [0;0;0];
            end
        end
    end
end
