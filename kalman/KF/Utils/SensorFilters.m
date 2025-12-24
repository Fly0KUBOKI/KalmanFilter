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
            SensorFilters.mex_kf_utils('sensor', 'reset');
        end

        function reset_zero()
            SensorFilters.mex_kf_utils('sensor', 'reset_zero');
        end

        function accel_config(cfg)
            SensorFilters.mex_kf_utils('sensor', 'accel_config', cfg);
        end

        function [a_filt, is_out] = accel(a_meas, a_expected)
            if nargout <= 1
                a_filt = SensorFilters.mex_kf_utils('sensor', 'accel', a_meas, a_expected);
                is_out = false;
            else
                try
                    [a_filt, is_out] = SensorFilters.mex_kf_utils('sensor', 'accel', a_meas, a_expected);
                catch ME
                    % Fallback: mex returned fewer outputs than expected
                    warning('SensorFilters.accel: mex_kf_utils accel did not return two outputs: %s', ME.message);
                    a_filt = SensorFilters.mex_kf_utils('sensor', 'accel', a_meas, a_expected);
                    is_out = false;
                end
            end
        end

        function [m_filt, is_out] = mag(m_meas, m_expected)
            if nargout <= 1
                m_filt = SensorFilters.mex_kf_utils('sensor', 'mag', m_meas, m_expected);
                is_out = false;
            else
                try
                    [m_filt, is_out] = SensorFilters.mex_kf_utils('sensor', 'mag', m_meas, m_expected);
                catch ME
                    warning('SensorFilters.mag: mex_kf_utils mag did not return two outputs: %s', ME.message);
                    m_filt = SensorFilters.mex_kf_utils('sensor', 'mag', m_meas, m_expected);
                    is_out = false;
                end
            end
        end

        function [pos_out, vel_out] = gps(gps_pos, dt)
            if nargout <= 1
                pos_out = SensorFilters.mex_kf_utils('sensor', 'gps', gps_pos, dt);
                vel_out = zeros(size(pos_out));
            else
                try
                    [pos_out, vel_out] = SensorFilters.mex_kf_utils('sensor', 'gps', gps_pos, dt);
                catch ME
                    warning('SensorFilters.gps: mex_kf_utils gps did not return two outputs: %s', ME.message);
                    pos_out = SensorFilters.mex_kf_utils('sensor', 'gps', gps_pos, dt);
                    vel_out = zeros(size(pos_out));
                end
            end
        end

        function [p, is_out] = baro(pressure)
            is_out = false;
            if nargout <= 1
                p = SensorFilters.mex_kf_utils('sensor', 'baro', pressure);
                is_out = false;
            else
                try
                    [p, is_out] = SensorFilters.mex_kf_utils('sensor', 'baro', pressure);
                catch ME
                    warning('SensorFilters.baro: mex_kf_utils baro did not return two outputs: %s', ME.message);
                    p = SensorFilters.mex_kf_utils('sensor', 'baro', pressure);
                    is_out = false;
                end
            end
        end

        function R = get_R(sensor_type)
            R = SensorFilters.mex_kf_utils('sensor', 'get_R', sensor_type);
            return;
        end

        function noise_estimate(sensor_type, innov, H, P)
            SensorFilters.mex_kf_utils('sensor', 'noise_estimate', sensor_type, innov, H, P);
            return;
        end

        function [dx_out, should_skip, was_attenuated] = divergence_check(sensor_name, innov, dx_in)
            [dx_out, should_skip, was_attenuated] = SensorFilters.mex_kf_utils('sensor', 'divergence_check', sensor_name, innov, dx_in);
            return;
        end

        function Pout = divergence_regularize(P)
            Pout = SensorFilters.mex_kf_utils('sensor', 'divergence_regularize', P);
            return;
        end
    end

    methods (Static, Access = private)
        function varargout = mex_kf_utils(mode, cmd, varargin)
            % Inline dispatcher (moved from mex_kf_utils.m)
            if nargin < 1
                error('mex_kf_utils:Usage', 'Usage: mex_kf_utils(''sensor''|''filter'', command, ...)');
            end

            mode = lower(mode);
            switch mode
                case 'sensor'
                    if nargin < 2, error('mex_kf_utils:MissingCommand', 'sensor mode requires a command'); end
                    if exist('mex_sensor_filter','file')==3 || exist('mex_sensor_filter','file')==2
                        [varargout{1:nargout}] = feval('mex_sensor_filter', cmd, varargin{:});
                        return;
                    else
                        error('mex_kf_utils:MissingMEX', 'Required MEX ''mex_sensor_filter'' not found. Run build_mex().');
                    end

                case 'filter'
                    if nargin < 2, error('mex_kf_utils:MissingCommand', 'filter mode requires a command'); end
                    if exist('mex_filter_utils','file')==3 || exist('mex_filter_utils','file')==2
                        [varargout{1:nargout}] = feval('mex_filter_utils', cmd, varargin{:});
                        return;
                    else
                        error('mex_kf_utils:MissingMEX', 'Required MEX ''mex_filter_utils'' not found. Run build_mex().');
                    end

                otherwise
                    error('mex_kf_utils:InvalidMode', 'Unknown mode "%s". Use ''sensor'' or ''filter''.', mode);
            end
        end
    end
end
