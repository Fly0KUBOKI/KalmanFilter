% DEPRECATED: SensorFilter.m is deprecated and moved to kalman/legacy_backup/KF/Utils/SensorFilter.m
% All calls should use direct filter instantiation (SensorAccelFilter, SensorMagFilter, etc)
% or MEX-based SensorFilters wrapper.
classdef SensorFilter
    methods (Static)
        function varargout = createAccelFilter(varargin)
            error('SensorFilter:Deprecated', ['SensorFilter.createAccelFilter is deprecated. '...
                'Use SensorAccelFilter directly: SensorAccelFilter(config) where config is a struct.']);
        end
        function varargout = createGyroFilter(varargin)
            error('SensorFilter:Deprecated', 'Gyro filter removed and not supported.');
        end
        function varargout = createMagFilter(varargin)
            error('SensorFilter:Deprecated', ['SensorFilter.createMagFilter is deprecated. '...
                'Use SensorMagFilter directly: SensorMagFilter(config).']);
        end
        function varargout = createGPSFilter(varargin)
            error('SensorFilter:Deprecated', ['SensorFilter.createGPSFilter is deprecated. '...
                'Use SensorGPSFilter directly: SensorGPSFilter(config).']);
        end
        function varargout = createBaroFilter(varargin)
            error('SensorFilter:Deprecated', ['SensorFilter.createBaroFilter is deprecated. '...
                'Use SensorBaroFilter directly: SensorBaroFilter(config).']);
        end
        function varargout = filterInnovation(varargin)
            error('SensorFilter:Deprecated', 'filterInnovation is deprecated; see legacy_backup for historical implementation.');
        end
        function varargout = shouldUpdate(varargin)
            error('SensorFilter:Deprecated', 'shouldUpdate is deprecated; see legacy_backup for historical implementation.');
        end
        function varargout = filterStateCorrection(varargin)
            error('SensorFilter:Deprecated', 'filterStateCorrection is deprecated; see legacy_backup for historical implementation.');
        end
    end
end
