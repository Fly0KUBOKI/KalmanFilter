% kalman_cpp_wrapper.m
% Minimal MATLAB-side convenience wrapper around the kalman_mex MEX-function.

function varargout = kalman_cpp_wrapper(cmd, varargin)
    switch lower(cmd)
        case 'new'
            varargout{1} = kalman_mex('new');
        case 'init'
            kalman_mex('init', varargin{:});
        case 'setsystem'
            kalman_mex('setSystem', varargin{:});
        case 'setobservationmatrix'
            kalman_mex('setObservationMatrix', varargin{:});
        case 'setprediction'
            kalman_mex('setPrediction', varargin{:});
        case 'setobservation'
            kalman_mex('setObservation', varargin{:});
        case 'update'
            kalman_mex('update', varargin{:});
        case 'get'
            varargout{1} = kalman_mex('get', varargin{:});
        case 'estimatenoise'
            kalman_mex('estimateNoise', varargin{:});
        case 'delete'
            kalman_mex('delete', varargin{:});
        otherwise
            error('Unknown command');
    end
end
