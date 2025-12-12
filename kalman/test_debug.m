% test_debug.m - デバッグ用スクリプト
try
    run_simulation(42, false);
    disp('SUCCESS: Simulation completed');
catch e
    fprintf('ERROR: %s\n', e.message);
    fprintf('Stack trace:\n');
    for i = 1:length(e.stack)
        fprintf('  %s (line %d)\n', e.stack(i).name, e.stack(i).line);
    end
end
