function compute_metrics_run()
    build_dir = fileparts(mfilename('fullpath'));
    kalman_root = fullfile(build_dir, '..', '..');
    kalman_root = matlab.lang.makeValidName(kalman_root); % ensure safe path string
    % Use explicit paths
    proj_root = fullfile(fileparts(build_dir),'..');
    proj_root = fullfile(fileparts(build_dir),'..');
    % Better: set proj_root to kalman folder provided by user structure
    proj_root = fullfile(fileparts(build_dir), '..');
    proj_root = fullfile(fileparts(build_dir), '..');
    % Resolve final project root as two levels up from this script
    proj_root = fullfile(build_dir, '..', '..');

    est_file = fullfile(proj_root, 'Results', 'estimation_01.csv');
    truth_file = fullfile(proj_root, 'GenerateData', 'truth_data.csv');

    if ~exist(est_file, 'file')
        fprintf('Estimation file not found: %s\n', est_file);
        return;
    end
    if ~exist(truth_file, 'file')
        fprintf('Truth file not found: %s\n', truth_file);
        return;
    end

    est = readtable(est_file);
    truth = readtable(truth_file);

    init_samples = 2000;
    idx = init_samples+1:height(est);

    posx_err = (est.px(idx) - truth.x(idx));
    posy_err = (est.py(idx) - truth.y(idx));
    posz_err = (est.pz(idx) - truth.z(idx));
    pos_err = sqrt(posx_err.^2 + posy_err.^2 + posz_err.^2);
    vel_err = sqrt((est.vx(idx) - truth.vx(idx)).^2 + (est.vy(idx) - truth.vy(idx)).^2 + (est.vz(idx) - truth.vz(idx)).^2);

    roll_err = rad2deg(wrapToPi(deg2rad(est.roll(idx) - truth.roll(idx))));
    pitch_err = rad2deg(wrapToPi(deg2rad(est.pitch(idx) - truth.pitch(idx))));
    yaw_err = rad2deg(wrapToPi(deg2rad(est.yaw(idx) - truth.yaw(idx))));

    metrics.pos_rmse = rms(pos_err);
    metrics.posx_rmse = rms(posx_err);
    metrics.posy_rmse = rms(posy_err);
    metrics.posz_rmse = rms(posz_err);
    metrics.vel_rmse = rms(vel_err);
    metrics.roll_rmse = rms(roll_err);
    metrics.pitch_rmse = rms(pitch_err);
    metrics.yaw_rmse = rms(yaw_err);

    metrics.ba_final = [est.ba_x(end), est.ba_y(end), est.ba_z(end)];
    metrics.bg_final = [est.bg_x(end), est.bg_y(end), est.bg_z(end)];

    if ismember('innov_norm', est.Properties.VariableNames)
        metrics.max_innov = max(est.innov_norm);
    else
        metrics.max_innov = 0;
    end
    if ismember('maha_dist', est.Properties.VariableNames)
        metrics.max_maha = max(est.maha_dist);
    else
        metrics.max_maha = 0;
    end

    metrics.has_nan = any(isnan(est.px) | isnan(est.py) | isnan(est.pz));
    metrics.has_inf = any(isinf(est.px) | isinf(est.py) | isinf(est.pz));

    disp(metrics);
end
