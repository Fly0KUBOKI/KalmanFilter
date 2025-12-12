function [p, v, q, ba, bg, P] = call_meukf_update_cpp(p, v, q, ba, bg, P, sensor, params, update_type)
    % C++ MEX経由でMEUKF更新を実行
    % 
    % 入力:
    %   p, v, q, ba, bg, P: 状態
    %   sensor: センサーデータ構造体
    %   params: パラメータ構造体
    %   update_type: 'accel', 'mag', 'baro', 'gps'
    %
    % 出力:
    %   更新後の状態
    
    % 状態構造体を作成
    state.p = p(:);
    state.v = v(:);
    state.q = q(:);
    state.ba = ba(:);
    state.bg = bg(:);
    state.P = P;
    
    % センサーデータ構造体（更新フラグ設定）
    sensor_data.accel = sensor.accel(:);
    sensor_data.gyro = sensor.gyro(:);
    sensor_data.mag = sensor.mag(:);
    sensor_data.gps_pos = sensor.gps_pos(:);
    sensor_data.alt_baro = sensor.alt_baro;
    sensor_data.dt = sensor.dt;
    
    % 更新フラグ
    sensor_data.update_accel = strcmp(update_type, 'accel');
    sensor_data.update_gyro = false;
    sensor_data.update_mag = strcmp(update_type, 'mag');
    sensor_data.update_gps = strcmp(update_type, 'gps');
    sensor_data.update_baro = strcmp(update_type, 'baro');
    sensor_data.update_zupt = strcmp(update_type, 'zupt');
    
    % パラメータ構造体
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
    
    % C++ MEX呼び出し
    try
        % mex_meukf_step_v2は1つの出力(state)のみを返す
        new_state = mex_meukf_step_v2(state, sensor_data, mex_params);
        
        % 状態を抽出
        p = new_state.p;
        v = new_state.v;
        q = new_state.q;
        ba = new_state.ba;
        bg = new_state.bg;
        P = new_state.P;
        
    catch ME
        warning('C++ MEUKF failed: %s', ME.message);
        % エラー時は状態を変更しない
    end
end
