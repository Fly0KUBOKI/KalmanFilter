% build_all.m
disp('Building eskf_core_mex...');
try
    mex -I../Common -I../ESKF mex_eskf_core.cpp ../ESKF/eskf_core.cpp -output eskf_core_mex
    disp('eskf_core_mex built successfully.');
catch e
    disp('Error building eskf_core_mex:');
    disp(e.message);
end

disp('Building mex_meukf_step_v2...');
try
    run('build_meukf.m');
catch e
    disp('Error building mex_meukf_step_v2:');
    disp(e.message);
end

disp('All builds complete.');
