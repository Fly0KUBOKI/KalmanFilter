function build_sensor_filter()
    try
        build_dir = fileparts(mfilename('fullpath'));
        cpp_root = fileparts(build_dir);
        mex_src_dir = fullfile(cpp_root, 'mex');
        bin_dir = fullfile(cpp_root, 'bin');
        
        if ~exist(bin_dir, 'dir')
            mkdir(bin_dir);
        end
        
        cd(mex_src_dir);
        
        fprintf('Building mex_sensor_filter...\n');
        % Use -v for verbose to see errors
        mex('-v', '-R2018a', 'mex_sensor_filter.cpp', '-outdir', bin_dir);
        
        fprintf('Build successful!\n');
    catch ME
        fprintf('Build failed: %s\n', ME.message);
        exit(1);
    end
end
