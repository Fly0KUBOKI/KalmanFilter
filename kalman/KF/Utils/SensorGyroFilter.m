% DEPRECATED: SensorGyroFilter.m is deprecated and moved to kalman/legacy_backup/KF/Utils/SensorGyroFilter.m
% Gyro filtering is no longer supported. Gyro measurements are processed by the INS/dead-reckoning path only.
classdef SensorGyroFilter < handle
    methods
        function obj = SensorGyroFilter(config)
            warning('SensorGyroFilter:Deprecated', ...
                'SensorGyroFilter is deprecated. Gyro filtering is no longer supported.');
        end
        function [w_out, is_outlier, info] = apply(obj, w_meas, w_expected)
            error('SensorGyroFilter:Deprecated', ...
                'SensorGyroFilter is deprecated and no longer supported.');
        end
        function noise_level = getNoiseLevel(obj)
            error('SensorGyroFilter:Deprecated', 'SensorGyroFilter is deprecated.');
        end
        function bias = getBiasEstimate(obj)
            error('SensorGyroFilter:Deprecated', 'SensorGyroFilter is deprecated.');
        end
    end
end
