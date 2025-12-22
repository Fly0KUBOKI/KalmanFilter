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
            if nargout <= 1
                a_filt = mex_sensor_filter('accel', a_meas, a_expected);
                is_out = false;
            else
                [a_filt, is_out] = mex_sensor_filter('accel', a_meas, a_expected);
            end
        end

        function [m_filt, is_out] = mag(m_meas, m_expected)
            if nargout <= 1
                m_filt = mex_sensor_filter('mag', m_meas, m_expected);
                is_out = false;
            else
                [m_filt, is_out] = mex_sensor_filter('mag', m_meas, m_expected);
            end
        end

        function [pos_out, vel_out] = gps(gps_pos, dt)
            if nargout <= 1
                pos_out = mex_sensor_filter('gps', gps_pos, dt);
                vel_out = zeros(size(pos_out));
            else
                [pos_out, vel_out] = mex_sensor_filter('gps', gps_pos, dt);
            end
        end

        function [p, is_out] = baro(pressure)
            is_out = false;
            if nargout <= 1
                p = mex_sensor_filter('baro', pressure);
                is_out = false;
            else
                [p, is_out] = mex_sensor_filter('baro', pressure);
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
    end
end
