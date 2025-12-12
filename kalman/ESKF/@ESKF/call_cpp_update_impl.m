function call_cpp_update_impl(obj, sensor_type, meas)
% C++更新統合実装（クラスメソッド）
    sensor_data = struct('accel', zeros(3,1), 'gyro', zeros(3,1), 'mag', zeros(3,1), ...
        'gps_pos', zeros(3,1), 'alt_baro', 0, 'dt', 0);
    params = struct('g', obj.g, 'mag_ref', [50;0;0], 'noise_accel', zeros(3,1), ...
        'noise_gyro', zeros(3,1), 'noise_ba', zeros(3,1), 'noise_bg', zeros(3,1), ...
        'noise_mag', zeros(3,1), 'noise_gps', zeros(3,1), 'noise_baro', 0, ...
        'alpha', 1e-3, 'beta', 2, 'kappa', 0);

    switch sensor_type
        case 'accel'
            R = diag(obj.noiseEstimator.getRnoise('accel')) * 1.5;
            sensor_data.accel = meas;
            params.noise_accel = R(1:3);
        case 'mag'
            R = diag(obj.noiseEstimator.getRnoise('mag')) * 1.5;
            sensor_data.mag = meas;
            params.noise_mag = R(1:3);
        case 'gps'
            R = diag(obj.noiseEstimator.getRnoise('gps'));
            sensor_data.gps_pos = meas;
            params.noise_gps = R(1:3);
        case 'baro'
            sensor_data.alt_baro = meas;
            params.noise_baro = obj.noiseEstimator.getRnoise('baro');
        case 'zupt'
            % ZUPT 用のノイズ設定（静止時の強い確信）
            R_diag = [0.01^2; 0.01^2; 0.01^2];
            params.noise_zupt = R_diag;
    end

    % 更新フラグを設定
    sensor_data.update_accel = strcmp(sensor_type, 'accel');
    sensor_data.update_gyro = false;
    sensor_data.update_mag = strcmp(sensor_type, 'mag');
    sensor_data.update_gps = strcmp(sensor_type, 'gps');
    sensor_data.update_baro = strcmp(sensor_type, 'baro');
    sensor_data.update_zupt = strcmp(sensor_type, 'zupt');

    % mex用パラメータ構築
    mex_params.g = params.g(:);
    mex_params.mag_ref = params.mag_ref(:);
    mex_params.noise_accel = params.noise_accel(:);
    mex_params.noise_gyro = params.noise_gyro(:);
    mex_params.noise_ba = params.noise_ba(:);
    mex_params.noise_bg = params.noise_bg(:);
    mex_params.noise_mag = params.noise_mag(:);
    mex_params.noise_gps = params.noise_gps(:);
    mex_params.noise_baro = params.noise_baro;
    if isfield(params, 'noise_zupt')
        mex_params.noise_zupt = params.noise_zupt(:);
    else
        mex_params.noise_zupt = zeros(3,1);
    end
    mex_params.alpha = params.alpha;
    mex_params.beta = params.beta;
    mex_params.kappa = params.kappa;

    % 状態構造体
    state.p = obj.p(:);
    state.v = obj.v(:);
    state.q = obj.q(:);
    state.ba = obj.ba(:);
    state.bg = obj.bg(:);
    state.P = obj.P;

    % C++ MEX呼び出し
    try
        new_state = mex_meukf_step_v2(state, sensor_data, mex_params);
        obj.p = new_state.p;
        obj.v = new_state.v;
        obj.q = new_state.q;
        obj.ba = new_state.ba;
        obj.bg = new_state.bg;
        obj.P = new_state.P;
    catch ME
        warning('ESKF:call_cpp_update_impl:Failed', 'C++ %s update failed: %s', sensor_type, ME.message);
    end
end
