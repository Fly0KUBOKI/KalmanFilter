function varargout = eskf_math(func_name, varargin)
    % eskf_math - MEX-only wrapper. MATLAB fallback removed (C++ implemented).
    if exist('mex_eskf_math', 'file') == 3
        [varargout{1:nargout}] = mex_eskf_math(func_name, varargin{:});
        return;
    else
        error('eskf_math:missing_mex', 'mex_eskf_math not found. MATLAB implementations removed; build the C++ MEX.');
    end
end
