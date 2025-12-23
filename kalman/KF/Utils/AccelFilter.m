classdef AccelFilter < handle
    % AccelFilter - DEPRECATED stub
    % This file has been disabled as part of Phase2 MEX migration.
    % Use SensorFilters.accel(...) (MEX) instead.

    methods
        function obj = AccelFilter(varargin)
            warning('AccelFilter:disabled', 'AccelFilter is disabled. Use SensorFilters.accel(a_meas, a_expected) instead.');
        end

        function varargout = filter(~, varargin)
            error('AccelFilter:disabled', 'AccelFilter.filter is disabled. Call SensorFilters.accel(a_meas, a_expected).');
        end

        function noise_level = getNoiseLevel(~)
            noise_level = NaN;
        end

        function setEMAAlpha(~, ~)
            warning('AccelFilter:disabled', 'AccelFilter.setEMAAlpha is disabled.');
        end
    end
end
