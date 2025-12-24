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
            mex_kf_utils('sensor', 'reset');
        end

        function reset_zero()
            mex_kf_utils('sensor', 'reset_zero');
        end

        function accel_config(cfg)
            mex_kf_utils('sensor', 'accel_config', cfg);
        end

        function [a_filt, is_out] = accel(a_meas, a_expected)
            if nargout <= 1
                a_filt = mex_kf_utils('sensor', 'accel', a_meas, a_expected);
                is_out = false;
            else
                try
                    [a_filt, is_out] = mex_kf_utils('sensor', 'accel', a_meas, a_expected);
                catch ME
                    % Fallback: mex returned fewer outputs than expected
                    warning('SensorFilters.accel: mex_kf_utils accel did not return two outputs: %s', ME.message);
                    a_filt = mex_kf_utils('sensor', 'accel', a_meas, a_expected);
                    is_out = false;
                end
            end
        end

        function [m_filt, is_out] = mag(m_meas, m_expected)
            if nargout <= 1
                m_filt = mex_kf_utils('sensor', 'mag', m_meas, m_expected);
                is_out = false;
            else
                try
                    [m_filt, is_out] = mex_kf_utils('sensor', 'mag', m_meas, m_expected);
                catch ME
                    warning('SensorFilters.mag: mex_kf_utils mag did not return two outputs: %s', ME.message);
                    m_filt = mex_kf_utils('sensor', 'mag', m_meas, m_expected);
                    is_out = false;
                end
            end
        end

        function [pos_out, vel_out] = gps(gps_pos, dt)
            if nargout <= 1
                pos_out = mex_kf_utils('sensor', 'gps', gps_pos, dt);
                vel_out = zeros(size(pos_out));
            else
                try
                    [pos_out, vel_out] = mex_kf_utils('sensor', 'gps', gps_pos, dt);
                catch ME
                    warning('SensorFilters.gps: mex_kf_utils gps did not return two outputs: %s', ME.message);
                    pos_out = mex_kf_utils('sensor', 'gps', gps_pos, dt);
                    vel_out = zeros(size(pos_out));
                end
            end
        end

        function [p, is_out] = baro(pressure)
            is_out = false;
            if nargout <= 1
                p = mex_kf_utils('sensor', 'baro', pressure);
                is_out = false;
            else
                try
                    [p, is_out] = mex_kf_utils('sensor', 'baro', pressure);
                catch ME
                    warning('SensorFilters.baro: mex_kf_utils baro did not return two outputs: %s', ME.message);
                    p = mex_kf_utils('sensor', 'baro', pressure);
                    is_out = false;
                end
            end
        end

        function R = get_R(sensor_type)
            R = mex_kf_utils('sensor', 'get_R', sensor_type);
            return;
        end

        function noise_estimate(sensor_type, innov, H, P)
            mex_kf_utils('sensor', 'noise_estimate', sensor_type, innov, H, P);
            return;
        end

        function [dx_out, should_skip, was_attenuated] = divergence_check(sensor_name, innov, dx_in)
            [dx_out, should_skip, was_attenuated] = mex_kf_utils('sensor', 'divergence_check', sensor_name, innov, dx_in);
            return;
        end

        function Pout = divergence_regularize(P)
            Pout = mex_kf_utils('sensor', 'divergence_regularize', P);
            return;
        end
    end
end
