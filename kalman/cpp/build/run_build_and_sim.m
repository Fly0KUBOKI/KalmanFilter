try
    % Change to build directory
    cd('C:/Users/takut/OneDrive/ドキュメント/MATLAB/KalmanFilter/kalman/cpp/build');

    % Ensure any loaded MEX are cleared to avoid file locks
    try
        clear mex;
    catch
    end
    pause(0.5);

    % Build MEX (all or specific)
    build_mex();

    % Clear old mex and add bin to path
    try
        clear mex;
    catch
    end
    addpath(fullfile(pwd,'..','bin'));

    % Return to kalman root and run simulation (use same seed)
    % From cpp/build -> ../../ should land in kalman/
    cd('../../');
    run_simulation(42, true);
catch ME
    disp(getReport(ME));
    exit(1);
end
exit(0);
