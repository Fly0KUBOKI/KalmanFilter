function build_mex()
    % BUILD_MEX  kalman_filter_core MEX関数をビルド
    % 
    % ESKFシミュレーション高速化のためのMEXビルドスクリプト
    %
    % 使用方法:
    %   build_mex()
    %
    % 必須:
    %   - C++コンパイラが設定済みであること (mex -setup C++)
    %   - cpp/KF/Core, cpp/Common/Math にC++実装があること

    fprintf('=== MEX Build for ESKF Acceleration ===\n');
    
    % パス設定
    cpp_root = fileparts(mfilename('fullpath'));
    mex_dir = fullfile(cpp_root, 'MEX');
    kf_dir = fullfile(cpp_root, 'KF', 'Core');
    common_dir = fullfile(cpp_root, 'Common', 'Math');
    
    % ディレクトリ存在確認
    if ~exist(mex_dir, 'dir')
        error('MEX directory not found: %s', mex_dir);
    end
    if ~exist(kf_dir, 'dir')
        error('KF/Core directory not found: %s', kf_dir);
    end
    if ~exist(common_dir, 'dir')
        error('Common/Math directory not found: %s', common_dir);
    end
    
    % コンパイラ設定確認
    try
        cc = mex.getCompilerConfigurations('C++', 'Selected');
        if isempty(cc)
            warning('C++ compiler not configured. Running mex -setup C++...');
            mex('-setup', 'C++');
        else
            fprintf('Using C++ compiler: %s\n', cc.Name);
        end
    catch
        warning('Failed to check compiler. Attempting build anyway...');
    end
    
    % ビルドディレクトリに移動
    original_dir = pwd;
    cd(mex_dir);
    
    build_success = true;
    built_count = 0;
    
    % コンパイルオプション設定
    compile_opts = {'-O', '-DNDEBUG'};
    if ispc
        compile_opts = [compile_opts, {'-DWIN32', '-D_CRT_SECURE_NO_WARNINGS'}];
    end
    
    % インクルードパス設定
    inc_kf = ['-I' kf_dir];
    inc_common = ['-I' common_dir];
    inc_cpp_root = ['-I' cpp_root];
    inc_args = {inc_kf, inc_common, inc_cpp_root};
    
    try
        % === 1. kalman_filter_core のビルド ===
        fprintf('\n=== [1/4] Building mex_kalman_filter_core ===\n');
        mex_file = 'mex_kalman_filter_core.cpp';
        
        if ~exist(mex_file, 'file')
            warning('KF MEX source not found: %s, skipping', mex_file);
        else
            fprintf('Sources: %s\n', mex_file);
            
            mex_args = [compile_opts, inc_args, {mex_file}];
            mex(mex_args{:});
            
            [~, mex_name, ~] = fileparts(mex_file);
            mex_output = [mex_name '.' mexext];
            if exist(mex_output, 'file')
                dest_file = fullfile(cpp_root, mex_output);
                copyfile(mex_output, dest_file, 'f');
                fprintf('✓ Built and copied: %s\n', mex_output);
                built_count = built_count + 1;
            else
                warning('MEX output file not found: %s', mex_output);
            end
        end
        
        % === 2. ukf_sigma_points のビルド ===
        fprintf('\n=== [2/4] Building mex_ukf_sigma_points ===\n');
        mex_file = 'mex_ukf_sigma_points.cpp';
        
        if ~exist(mex_file, 'file')
            warning('UKF MEX source not found: %s, skipping', mex_file);
        else
            fprintf('Sources: %s\n', mex_file);
            
            mex_args = [compile_opts, inc_args, {mex_file}];
            mex(mex_args{:});
            
            [~, mex_name, ~] = fileparts(mex_file);
            mex_output = [mex_name '.' mexext];
            if exist(mex_output, 'file')
                dest_file = fullfile(cpp_root, mex_output);
                copyfile(mex_output, dest_file, 'f');
                fprintf('✓ Built and copied: %s\n', mex_output);
                built_count = built_count + 1;
            else
                warning('MEX output file not found: %s', mex_output);
            end
        end
        
        % === 3. eskf_core のビルド ===
        fprintf('\n=== [3/4] Building mex_eskf_core ===\n');
        mex_file = 'mex_eskf_core.cpp';
        eskf_core_cpp = fullfile(cpp_root, 'ESKF', 'eskf_core.cpp');
        eskf_dir = fullfile(cpp_root, 'ESKF');
        inc_eskf = ['-I' eskf_dir];
        
        if ~exist(mex_file, 'file')
            warning('ESKF MEX source not found: %s, skipping', mex_file);
        elseif ~exist(eskf_core_cpp, 'file')
            warning('ESKF core implementation not found: %s, skipping', eskf_core_cpp);
        else
            fprintf('Sources: %s, %s\n', mex_file, eskf_core_cpp);
            
            % kalman_filter_core.cpp is now header-only, removed from build
            mex_args = [compile_opts, inc_args, {inc_eskf, mex_file, eskf_core_cpp}];
            mex(mex_args{:});
            
            [~, mex_name, ~] = fileparts(mex_file);
            mex_output = [mex_name '.' mexext];
            if exist(mex_output, 'file')
                dest_file = fullfile(cpp_root, mex_output);
                copyfile(mex_output, dest_file, 'f');
                fprintf('✓ Built and copied: %s\n', mex_output);
                built_count = built_count + 1;
            else
                warning('MEX output file not found: %s', mex_output);
            end
        end
        
        % === 4. quaternion_lib のビルド ===
        fprintf('\n=== [4/6] Building mex_quaternion_lib ===\n');
        mex_file = 'mex_quaternion_lib.cpp';
        
        if ~exist(mex_file, 'file')
            warning('Quaternion MEX source not found: %s, skipping', mex_file);
        else
            fprintf('Sources: %s\n', mex_file);
            
            mex_args = [compile_opts, inc_args, {mex_file}];
            mex(mex_args{:});
            
            [~, mex_name, ~] = fileparts(mex_file);
            mex_output = [mex_name '.' mexext];
            if exist(mex_output, 'file')
                dest_file = fullfile(cpp_root, mex_output);
                copyfile(mex_output, dest_file, 'f');
                fprintf('✓ Built and copied: %s\n', mex_output);
                built_count = built_count + 1;
            else
                warning('MEX output file not found: %s', mex_output);
            end
        end
        
        % === 5. ukf_update のビルド ===
        fprintf('\n=== [5/6] Building mex_ukf_update ===\n');
        mex_file = 'mex_ukf_update.cpp';
        
        if ~exist(mex_file, 'file')
            warning('UKF update MEX source not found: %s, skipping', mex_file);
        else
            fprintf('Sources: %s\n', mex_file);
            
            ukf_dir = fullfile(cpp_root, 'UKF');
            inc_ukf = ['-I' ukf_dir];
            
            mex_args = [compile_opts, inc_args, {inc_ukf, mex_file}];
            mex(mex_args{:});
            
            [~, mex_name, ~] = fileparts(mex_file);
            mex_output = [mex_name '.' mexext];
            if exist(mex_output, 'file')
                dest_file = fullfile(cpp_root, mex_output);
                copyfile(mex_output, dest_file, 'f');
                fprintf('✓ Built and copied: %s\n', mex_output);
                built_count = built_count + 1;
            else
                warning('MEX output file not found: %s', mex_output);
            end
        end
        
        % === 6. common_lib のビルド (スキップ - オプション) ===
        fprintf('\n=== [6/6] Skipping mex_common_lib (deprecated) ===\n');
        % mex_common_lib は古い FixedMatrix API を使用しているためスキップ
        
    catch ME
        build_success = false;
        fprintf('\n✗ Build failed!\n');
        fprintf('Error: %s\n', ME.message);
        fprintf('\nTroubleshooting:\n');
        fprintf('1. Run "mex -setup C++" to configure compiler\n');
        fprintf('2. Ensure Visual Studio or MinGW is installed\n');
        fprintf('3. Check that all source files exist\n');
        fprintf('4. Try running with -v flag for verbose output\n');
    end
    
    % 元のディレクトリに戻る
    cd(original_dir);
    
    if build_success
        fprintf('\n=== Build Complete ===\n');
        fprintf('Successfully built %d MEX file(s)\n', built_count);
        fprintf('\nMEX acceleration will be automatically used when available.\n');
        fprintf('Test with:\n');
        fprintf('  >> check_mex_usage\n');
        fprintf('  >> test_mex_kalman_filter_core\n');
    else
        error('MEX build failed. See error messages above.');
    end
end
