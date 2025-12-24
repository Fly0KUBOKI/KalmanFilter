function varargout = mex_kf_utils(mode, cmd, varargin)
% MEX_KF_UTILS  Unified MATLAB dispatcher for KF-related MEXs
% Usage:
%  mex_kf_utils('sensor', command, ...)
%  mex_kf_utils('filter', command, ...)
%
% This file centralizes calls to two compiled MEX modules used by
% kalman/KF/Utils: `mex_sensor_filter` and `mex_filter_utils`.
% It forwards calls to the appropriate MEX and provides clearer
% single-call entry for MATLAB-side code.

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
