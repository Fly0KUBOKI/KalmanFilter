% DEPRECATED: SensorFilterFactory.m is deprecated and moved to kalman/legacy_backup/KF/Utils/SensorFilterFactory.m
% All calls should use direct filter instantiation (SensorAccelFilter, SensorMagFilter, etc)
% or MEX-based SensorFilters wrapper.
classdef SensorFilterFactory < handle
    properties (Constant)
        SENSOR_ACCEL = 'accel'
        SENSOR_GYRO = 'gyro'
        SENSOR_MAG = 'mag'
        SENSOR_GPS = 'gps'
        SENSOR_BARO = 'baro'
    end
    
    methods (Static)
        function filter = createFilter(sensor_type, varargin)
            error('SensorFilterFactory:Deprecated', ['SensorFilterFactory.createFilter is deprecated. '...
                'Use direct filter instantiation: '...
                'SensorAccelFilter(config), SensorMagFilter(config), SensorGPSFilter(config), or SensorBaroFilter(config).']);
        end
    end
end
