function select_mex_compiler(compiler)
% SELECT_MEX_COMPILER  Select and verify C++ compiler for MEX.
%
% Usage:
%  select_mex_compiler('mingw')  - select MinGW
%  select_mex_compiler('msvc')   - select Microsoft Visual C++
%  select_mex_compiler()         - interactive selection (mex -setup C++)

if nargin < 1 || isempty(compiler)
    compiler = '';
end

compiler = lower(char(compiler));

fprintf('\n=== Select C++ Compiler ===\n\n');

% List installed compilers
fprintf('Available compilers:\n');
try
    ccs = mex.getCompilerConfigurations('C++','Installed');
    if isempty(ccs)
        fprintf('  (none found)\n');
    else
        for k = 1:numel(ccs)
            fprintf('  [%d] %s\n', k, ccs(k).Name);
        end
    end
catch
    fprintf('  (unable to list)\n');
end

% Select based on input or interactive
if isempty(compiler)
    % Interactive mode
    fprintf('\nRunning: mex -setup C++\n');
    try
        evalc('mex -setup C++');
    catch
        fprintf('✗ mex -setup failed\n');
        return;
    end
else
    % Programmatic selection
    switch compiler
        case {'mingw', 'gcc'}
            % Try MinGW XML
            xml = fullfile(matlabroot, 'bin', 'win64', 'mexopts', 'mingw64_g++.xml');
            if isfile(xml)
                try
                    evalc(sprintf('mex -setup:''%s'' C++', xml));
                    fprintf('Selected MinGW from: %s\n', xml);
                catch
                    fprintf('✗ Failed to setup MinGW from XML\n');
                    return;
                end
            else
                fprintf('✗ MinGW XML not found at: %s\n', xml);
                return;
            end
        case {'msvc', 'visual', 'vc'}
            % Try MSVC
            selected = false;
            try
                ccs = mex.getCompilerConfigurations('C++','Installed');
                if ~isempty(ccs)
                    for k = 1:numel(ccs)
                        name = '';
                        try
                            name = ccs(k).Name;
                        catch
                            continue;
                        end
                        if contains(name, 'Visual', 'IgnoreCase', true) || ...
                           contains(name, 'Microsoft', 'IgnoreCase', true)
                            % Try the standard call, then the object-style call if needed
                            try
                                try
                                    mex.setCompilerConfigurations(ccs(k));
                                catch e1
                                    % Some MATLAB versions require mex() object syntax
                                    try
                                        mex().setCompilerConfigurations(ccs(k));
                                    catch e2
                                        fprintf('✗ Error setting compiler (both attempts failed): %s | %s\n', e1.message, e2.message);
                                        continue;
                                    end
                                end
                                fprintf('Selected: %s\n', name);
                                selected = true;
                                break;
                            catch e
                                fprintf('✗ Error setting compiler: %s\n', e.message);
                                continue;
                            end
                        end
                    end
                end
            catch e
                fprintf('✗ Error getting compiler configurations: %s\n', e.message);
            end
            if ~selected
                    % Fallback: try mexopts XML files for MSVC
                    xml_dir = fullfile(matlabroot, 'bin', 'win64', 'mexopts');
                    if exist(xml_dir,'dir')
                        xml_files = [dir(fullfile(xml_dir,'*vc*.xml')); dir(fullfile(xml_dir,'*msvc*.xml'))];
                        for xi = 1:numel(xml_files)
                            xml_path = fullfile(xml_dir, xml_files(xi).name);
                            try
                                evalc(sprintf('mex -setup:''%s'' C++', xml_path));
                            catch
                            end
                            % verify
                            try
                                sel = mex.getCompilerConfigurations('C++','Selected');
                                if ~isempty(sel) && (contains(sel.Name,'Visual','IgnoreCase',true) || contains(sel.Name,'Microsoft','IgnoreCase',true))
                                    fprintf('✓ MSVC selected via XML: %s\n', sel.Name);
                                    selected = true; break;
                                end
                            catch
                            end
                        end
                    end
                    if ~selected
                        fprintf('✗ MSVC not found in installed list\n');
                        return;
                    end
            end
        otherwise
            fprintf('✗ Unknown compiler: %s\n', compiler);
            return;
    end
end

% Verify selection
fprintf('\nVerifying selection:\n');
try
    sel = mex.getCompilerConfigurations('C++','Selected');
    if isempty(sel)
        fprintf('✗ No compiler selected\n');
        return;
    end
    fprintf('✓ Selected: %s\n', sel.Name);
catch
    fprintf('✗ Verification failed\n');
    return;
end

fprintf('\nDone.\n\n');

end
