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
            if exist('mex_sensor_filter','file')
                mex_sensor_filter('reset');
            else
                error('SensorFilters:MissingMEX','mex_sensor_filter not found on MATLAB path');
            end
        end

        function reset_zero()
            if exist('mex_sensor_filter','file')
                mex_sensor_filter('reset_zero');
            else
                error('SensorFilters:MissingMEX','mex_sensor_filter not found on MATLAB path');
            end
        end

        function accel_config(cfg)
            if exist('mex_sensor_filter','file')
                mex_sensor_filter('accel_config', cfg);
            else
                error('SensorFilters:MissingMEX','mex_sensor_filter not found on MATLAB path');
            end
        end

        function [a_filt, is_out] = accel(a_meas, a_expected)
            force_matlab_env = getenv('FORCE_MATLAB_FILTERS');
            force_matlab = ~isempty(force_matlab_env) && strcmp(force_matlab_env, '1');
            if force_matlab || exist('mex_sensor_filter','file')~=3
                % Use MATLAB fallback filter implementation
                f = SensorFilter.createAccelFilter();
                if nargout <= 1
                    a_filt = f.apply(a_meas, a_expected);
                else
                    [a_filt, is_out, ~] = f.apply(a_meas, a_expected);
                end
                return;
            end
            if nargout <= 1
                a_filt = mex_sensor_filter('accel', a_meas, a_expected);
            else
                [a_filt, is_out] = mex_sensor_filter('accel', a_meas, a_expected);
            end
        end

        function [m_filt, is_out] = mag(m_meas, m_expected)
            force_matlab_env = getenv('FORCE_MATLAB_FILTERS');
            force_matlab = ~isempty(force_matlab_env) && strcmp(force_matlab_env, '1');
            if force_matlab || exist('mex_sensor_filter','file')~=3
                f = SensorFilter.createMagFilter();
                if nargout <= 1
                    m_filt = f.apply(m_meas);
                else
                    [m_filt, is_out, ~] = f.apply(m_meas);
                end
                return;
            end
            if nargout <= 1
                m_filt = mex_sensor_filter('mag', m_meas, m_expected);
            else
                [m_filt, is_out] = mex_sensor_filter('mag', m_meas, m_expected);
            end
        end

        function [pos_out, vel_out] = gps(gps_pos, dt)
            force_matlab_env = getenv('FORCE_MATLAB_FILTERS');
            force_matlab = ~isempty(force_matlab_env) && strcmp(force_matlab_env, '1');
            if force_matlab || exist('mex_sensor_filter','file')~=3
                f = SensorFilter.createGPSFilter();
                if nargout <= 1
                    pos_out = f.apply(gps_pos);
                else
                    [pos_out, is_out, ~] = f.apply(gps_pos);
                    vel_out = zeros(size(pos_out));
                end
                return;
            end
            if nargout <= 1
                pos_out = mex_sensor_filter('gps', gps_pos, dt);
            else
                [pos_out, vel_out] = mex_sensor_filter('gps', gps_pos, dt);
            end
        end

        function [p, is_out] = baro(pressure)
            force_matlab_env = getenv('FORCE_MATLAB_FILTERS');
            force_matlab = ~isempty(force_matlab_env) && strcmp(force_matlab_env, '1');
            is_out = false;
            if force_matlab || exist('mex_sensor_filter','file')~=3
                f = SensorFilter.createBaroFilter();
                if nargout <= 1
                    p = f.apply(pressure);
                    is_out = false;
                else
                    [p, is_out, ~] = f.apply(pressure);
                end
                return;
            end
            if nargout <= 1
                p = mex_sensor_filter('baro', pressure);
                is_out = false;
            else
                try
                    [p, is_out] = mex_sensor_filter('baro', pressure);
                catch
                    % MEX didn't provide a second output — fallback to single output
                    p = mex_sensor_filter('baro', pressure);
                    is_out = false;
                end
            end
        end

        function R = get_R(sensor_type)
            force_matlab_env = getenv('FORCE_MATLAB_FILTERS');
            force_matlab = ~isempty(force_matlab_env) && strcmp(force_matlab_env, '1');
            if ~force_matlab && exist('mex_sensor_filter','file')==3
                R = mex_sensor_filter('get_R', sensor_type);
                return;
            end
            % MATLAB fallback defaults (match NoiseEstimator defaults)
            switch sensor_type
                case 'accel'
                    R = diag(ones(3,1) * 0.01);
                case 'gyro'
                    R = diag(ones(3,1) * (deg2rad(0.1)^2));
                case 'mag'
                    R = diag(ones(3,1) * 5.0^2);
                case 'gps'
                    R = diag([3^2; 3^2; 5^2]);
                case 'baro'
                    R = 1.0^2;
                otherwise
                    error('Unknown sensor type: %s', sensor_type);
            end
        end

        function noise_estimate(sensor_type, innov, H, P)
            force_matlab_env = getenv('FORCE_MATLAB_FILTERS');
            force_matlab = ~isempty(force_matlab_env) && strcmp(force_matlab_env, '1');
            if ~force_matlab && exist('mex_sensor_filter','file')==3
                mex_sensor_filter('noise_estimate', sensor_type, innov, H, P);
                return;
            end
            % MATLAB fallback: no-op (NoiseEstimator MATLAB shim handles local estimates)
            return;
        end

        function [dx_out, should_skip, was_attenuated] = divergence_check(sensor_name, innov, dx_in)
            force_matlab_env = getenv('FORCE_MATLAB_FILTERS');
            force_matlab = ~isempty(force_matlab_env) && strcmp(force_matlab_env, '1');
            if force_matlab || exist('mex_sensor_filter','file')~=3
                persistent div_guard
                if isempty(div_guard)
                    div_guard = DivergenceGuard();
                end
                [dx_out, should_skip, was_attenuated] = div_guard.check_and_attenuate_update(sensor_name, innov, dx_in, struct());
                return;
            end
            [dx_out, should_skip, was_attenuated] = mex_sensor_filter('divergence_check', sensor_name, innov, dx_in);
        end

        function Pout = divergence_regularize(P)
            force_matlab_env = getenv('FORCE_MATLAB_FILTERS');
            force_matlab = ~isempty(force_matlab_env) && strcmp(force_matlab_env, '1');
            if force_matlab || exist('mex_sensor_filter','file')~=3
                persistent div_guard
                if isempty(div_guard)
                    div_guard = DivergenceGuard();
                end
                Pout = div_guard.regularize_covariance(P);
                return;
            end
            Pout = mex_sensor_filter('divergence_regularize', P);
        end
    end
end
