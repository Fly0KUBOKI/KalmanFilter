% build_filter_utils.m
% Build mex_filter_utils MEX function
%
% Usage:
%   cd kalman/cpp/MEX
%   build_filter_utils

fprintf('=== Building mex_filter_utils ===\n');

% Get current directory
mex_dir = fileparts(mfilename('fullpath'));
cpp_root = fileparts(mex_dir);

% Check if source file exists
mex_file = fullfile(mex_dir, 'mex_filter_utils.cpp');
if ~exist(mex_file, 'file')
    error('Source file not found: %s', mex_file);
end

% Include paths (mex_filter_utils.cpp uses ../Common/ paths)
inc_common = ['-I' fullfile(cpp_root, 'Common')];
inc_args = {inc_common};

% Compile options
compile_opts = {'-O', '-DNDEBUG'};
if ispc
    compile_opts = [compile_opts, {'-DWIN32', '-D_CRT_SECURE_NO_WARNINGS'}];
end

try
    % Build MEX function
    fprintf('Compiling mex_filter_utils...\n');
    mex_args = [compile_opts, inc_args, {'-output', 'mex_filter_utils'}, {mex_file}];
    mex(mex_args{:});
    
    % Check output
    mex_output = ['mex_filter_utils.' mexext];
    if exist(mex_output, 'file')
        fprintf('Successfully built: %s\n', mex_output);
        fprintf('Output location: %s\n', fullfile(mex_dir, mex_output));
    else
        error('Output file not generated: %s', mex_output);
    end
catch ME
    error('Build failed: %s', ME.message);
end

fprintf('=== Build Complete ===\n');
