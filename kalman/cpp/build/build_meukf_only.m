% Quick build for mex_meukf_step_v2 only
fprintf('=== Building mex_meukf_step_v2 only ===\n');

build_dir = fileparts(mfilename('fullpath'));
cpp_root = fileparts(build_dir);
mex_src_dir = fullfile(cpp_root, 'mex');
inc_dir = fullfile(cpp_root, 'include');
bin_dir = fullfile(cpp_root, 'bin');

cd(mex_src_dir);

compile_opts = {'-O', '-DNDEBUG', '-DWIN32', '-D_CRT_SECURE_NO_WARNINGS'};
inc_kf_core = ['-I' fullfile(inc_dir, 'KF')];
inc_ekf_core = ['-I' fullfile(inc_dir, 'EKF')];
inc_eskf = ['-I' fullfile(inc_dir, 'ESKF')];
inc_ukf_core = ['-I' fullfile(inc_dir, 'UKF')];
inc_common = ['-I' fullfile(inc_dir, 'Common')];
inc_meukf = ['-I' fullfile(cpp_root, 'MEUKF')];
inc_args = {inc_kf_core, inc_ekf_core, inc_eskf, inc_ukf_core, inc_common, inc_meukf};

meukf_core_cpp = fullfile(cpp_root, 'MEUKF', 'meukf_core.cpp');

fprintf('Compiling mex_meukf_step.cpp + meukf_core.cpp...\n');
mex_args = [{'mex_meukf_step.cpp', meukf_core_cpp}, compile_opts, inc_args, ...
            {'-outdir', bin_dir, '-output', 'mex_meukf_step_v2'}];
try
    mex(mex_args{:});
    fprintf('SUCCESS: Built mex_meukf_step_v2.mexw64\n');
    cd(build_dir);
catch ME
    fprintf('ERROR: %s\n', ME.message);
    cd(build_dir);
    rethrow(ME);
end
