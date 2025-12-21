% compare_results.m - Compare MEX vs MATLAB estimation results
function compare_results()
    truth = readtable('GenerateData/truth_data.csv');
    est_mex = readtable('Results/estimation.csv');
    
    % Check if MATLAB result exists
    matlab_file = 'Results/estimation_matlab.csv';
    if exist(matlab_file, 'file')
        est_mat = readtable(matlab_file);
        has_matlab = true;
    else
        has_matlab = false;
    end

    t_start = 5.0; 
    idx = truth.time >= t_start;

    % MEX results
    err_roll_mex = truth.roll(idx) - est_mex.roll(idx);
    err_pitch_mex = truth.pitch(idx) - est_mex.pitch(idx);
    err_yaw_mex = wrapTo180(truth.yaw(idx) - est_mex.yaw(idx));
    rmse_roll_mex = sqrt(mean(err_roll_mex.^2));
    rmse_pitch_mex = sqrt(mean(err_pitch_mex.^2));
    rmse_yaw_mex = sqrt(mean(err_yaw_mex.^2));

    fprintf('=== MEX Results (estimation.csv) ===\n');
    fprintf('Roll RMSE:  %.4f deg\n', rmse_roll_mex);
    fprintf('Pitch RMSE: %.4f deg\n', rmse_pitch_mex);
    fprintf('Yaw RMSE:   %.4f deg\n', rmse_yaw_mex);
    
    if rmse_roll_mex < 0.5 && rmse_pitch_mex < 0.5
        fprintf('MEX Result: PASS\n');
    else
        fprintf('MEX Result: FAIL (target < 0.5 deg)\n');
    end

    if has_matlab
        % MATLAB results
        err_roll_mat = truth.roll(idx) - est_mat.roll(idx);
        err_pitch_mat = truth.pitch(idx) - est_mat.pitch(idx);
        err_yaw_mat = wrapTo180(truth.yaw(idx) - est_mat.yaw(idx));
        rmse_roll_mat = sqrt(mean(err_roll_mat.^2));
        rmse_pitch_mat = sqrt(mean(err_pitch_mat.^2));
        rmse_yaw_mat = sqrt(mean(err_yaw_mat.^2));

        fprintf('\n=== MATLAB Results (estimation_matlab.csv) ===\n');
        fprintf('Roll RMSE:  %.4f deg\n', rmse_roll_mat);
        fprintf('Pitch RMSE: %.4f deg\n', rmse_pitch_mat);
        fprintf('Yaw RMSE:   %.4f deg\n', rmse_yaw_mat);
        
        if rmse_roll_mat < 0.5 && rmse_pitch_mat < 0.5
            fprintf('MATLAB Result: PASS\n');
        else
            fprintf('MATLAB Result: FAIL (target < 0.5 deg)\n');
        end

        fprintf('\n=== Difference ===\n');
        fprintf('Roll diff:  %.4f deg\n', abs(rmse_roll_mex - rmse_roll_mat));
        fprintf('Pitch diff: %.4f deg\n', abs(rmse_pitch_mex - rmse_pitch_mat));
    else
        fprintf('\n(estimation_matlab.csv not found - no comparison available)\n');
    end
end
