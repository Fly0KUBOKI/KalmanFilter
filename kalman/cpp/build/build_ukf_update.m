function build_ukf_update()
    % BUILD_UKF_UPDATE  Build only mex_ukf_update
    
    fprintf('=== MEX Build for mex_ukf_update ===\n');
    
    build_dir = fileparts(mfilename('fullpath'));
    cpp_root = fileparts(build_dir);
    mex_src_dir = fullfile(cpp_root, 'mex');
    inc_dir = fullfile(cpp_root, 'include');
    bin_dir = fullfile(cpp_root, 'bin');
    
    if ~exist(bin_dir, 'dir'), mkdir(bin_dir); end
    
    % Include paths
    inc_kf_core = ['-I' fullfile(inc_dir, 'KF')];
    inc_ekf_core = ['-I' fullfile(inc_dir, 'EKF')];
    inc_eskf = ['-I' fullfile(inc_dir, 'ESKF')];
    inc_ukf_core = ['-I' fullfile(inc_dir, 'UKF')];
    inc_common = ['-I' fullfile(inc_dir, 'Common')];
    inc_args = {inc_kf_core, inc_ekf_core, inc_eskf, inc_ukf_core, inc_common};
    
    compile_opts = {'-v', '-O', '-DNDEBUG'};
    if ispc
        compile_opts = [compile_opts, {'-DWIN32', '-D_CRT_SECURE_NO_WARNINGS'}];
    end
    
    original_dir = pwd;
    cd(mex_src_dir);
    
    try
        fprintf('Compiling mex_ukf_update.cpp...\n');
        mex(compile_opts{:}, inc_args{:}, 'mex_ukf_update.cpp');
        
        mex_output = ['mex_ukf_update.' mexext];
        if exist(mex_output, 'file')
            copyfile(mex_output, fullfile(bin_dir, mex_output), 'f');
            fprintf('Success: %s\n', fullfile(bin_dir, mex_output));
        else
            error('Output file not generated');
        end
    catch ME
        fprintf('Error: %s\n', ME.message);
    end
    
    cd(original_dir);
end
