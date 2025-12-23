classdef SensorAccelFilter < handle
    % Deprecated stub: functionality moved to SensorFilters (MEX).
    methods
        function obj = SensorAccelFilter(varargin)
            warning('SensorAccelFilter:disabled','SensorAccelFilter is disabled. Use SensorFilters.accel(a_meas,a_expected).');
        end
        function varargout = apply(~, varargin)
            error('SensorAccelFilter:disabled','SensorAccelFilter.apply is disabled. Call SensorFilters.accel(a_meas,a_expected).');
        end
        function noise_level = getNoiseLevel(~)
            noise_level = NaN;
        end
    end
end
