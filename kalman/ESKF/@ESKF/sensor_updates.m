function varargout = sensor_updates(obj, method, varargin)
    % センサー更新統合メソッド
    % 使用例:
    %   obj.sensor_updates('accel', a_meas)
    %   obj.sensor_updates('mag', m_meas)
    %   obj.sensor_updates('gps', lat, lon, alt, k)
    %   obj.sensor_updates('baro', pressure)
    
    switch method
        case 'accel'
            update_accel_impl(obj, varargin{1});
        case 'mag'
            update_mag_impl(obj, varargin{1});
        case 'gps'
            update_gps_impl(obj, varargin{1}, varargin{2}, varargin{3}, varargin{4});
        case 'baro'
            update_baro_impl(obj, varargin{1});
        otherwise
            error('Unknown sensor update method: %s', method);
    end
end

function update_accel_impl(obj, a_meas)
    % 加速度更新 (変更検知追加)
    
    % 変更検知
    if norm(a_meas - obj.prev_accel) <= obj.buffer_tolerance
        return;  % データが前回と同じならスキップ
    end
    obj.prev_accel = a_meas;

    if ~isempty(obj.w_body) && norm(obj.w_body) > 1.5; return; end
    [a_corrected, is_outlier, ~] = obj.sensor_filters.accel.apply(a_meas, zeros(3,1));
    if any(isnan(a_corrected)) || is_outlier; return; end
    a_norm = norm(a_corrected);
    if a_norm < 0.1 || abs(a_norm - 9.81) > 3.0; return; end
    do_cpp_update(obj, 'accel', a_corrected);
end

function update_mag_impl(obj, m_meas)
    % 磁気計更新 (Phase 1: 変更検知追加)
    
    % 変更検知 (SensorDataBuffer統合)
    if norm(m_meas - obj.prev_mag) <= obj.buffer_tolerance
        return;  % データが前回と同じならスキップ
    end
    obj.prev_mag = m_meas;

    [m_filtered, is_outlier, ~] = obj.sensor_filters.mag.apply(m_meas);
    if any(isnan(m_filtered)) || is_outlier; return; end
    do_cpp_update(obj, 'mag', m_filtered);
end

function update_gps_impl(obj, lat, lon, alt, k)
    % GPS更新 (Phase 1: 変更検知追加)
    
    % 変更検知 (SensorDataBuffer統合)
    d_lat = abs(lat - obj.prev_gps_lat);
    d_lon = abs(lon - obj.prev_gps_lon);
    d_alt = abs(alt - obj.prev_gps_alt);
    if (d_lat <= obj.buffer_tolerance) && (d_lon <= obj.buffer_tolerance) && (d_alt <= obj.buffer_tolerance)
        return;  % データが前回と同じならスキップ
    end
    obj.prev_gps_lat = lat;
    obj.prev_gps_lon = lon;
    obj.prev_gps_alt = alt;
    
    lat0 = obj.gps_origin(1);
    lon0 = obj.gps_origin(2);
    alt0 = obj.gps_origin(3);
    y_m = (lat - lat0) / (9.0e-6);
    x_m = (lon - lon0) / (9.0e-6 / cosd(lat0));
    z_m = alt - alt0;
    z_gps = [y_m; x_m; -z_m];
    do_cpp_update(obj, 'gps', z_gps);
end

function update_baro_impl(obj, pressure)
    % 気圧計更新 (Phase 1: 変更検知追加)
    
    % 変更検知 (SensorDataBuffer統合)
    if abs(pressure - obj.prev_baro) <= obj.buffer_tolerance
        return;  % データが前回と同じならスキップ
    end
    obj.prev_baro = pressure;
    
    [alt_baro, is_outlier, ~] = obj.sensor_filters.baro.apply(pressure);
    if any(isnan(alt_baro)) || is_outlier; return; end

    weight_factor = 1.0 / obj.baro_weight;
    obj.P(3,3) = obj.P(3,3) * weight_factor;
    do_cpp_update(obj, 'baro', alt_baro);
    obj.P(3,3) = obj.P(3,3) / weight_factor;
end

function do_cpp_update(obj, sensor_type, meas)
    % 内部ヘルパー: C++ MEX に直接渡して更新を行う
    sensor_data = struct('accel', zeros(3,1), 'gyro', zeros(3,1), 'mag', zeros(3,1), ...
        'gps_pos', zeros(3,1), 'alt_baro', 0, 'dt', obj.dt);
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
            R_diag = [0.01^2; 0.01^2; 0.01^2];
            params.noise_zupt = R_diag;
    end

    % 更新フラグ
    sensor_data.update_accel = strcmp(sensor_type, 'accel');
    sensor_data.update_gyro = false;
    sensor_data.update_mag = strcmp(sensor_type, 'mag');
    sensor_data.update_gps = strcmp(sensor_type, 'gps');
    sensor_data.update_baro = strcmp(sensor_type, 'baro');
    sensor_data.update_zupt = strcmp(sensor_type, 'zupt');

    % mex パラメータ整形
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

    % 状態構築
    % 上流ログ: state が破損していないか簡易 trace を残す
    try
        % ログはデフォルトで無効。必要な場合は環境変数 ENABLE_STATE_TRACE=1 を設定する。
        trace_enabled = strcmp(getenv('ENABLE_STATE_TRACE'),'1');
        if trace_enabled
            outdir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Results'); if ~exist(outdir,'dir'), mkdir(outdir); end
            trace_file = fullfile(outdir, 'state_trace.txt');
            fid = fopen(trace_file, 'a');
            if fid ~= -1
                fprintf(fid, '%s sensor=%s dt=%g p_max=%g v_max=%g q_norm=%g P_max=%g\n', ...
                    datestr(now,'yyyy-mm-dd HH:MM:SS.FFF'), sensor_type, obj.dt, max(abs(obj.p(:))), max(abs(obj.v(:))), norm(obj.q(:)), max(abs(obj.P(:))));
                fclose(fid);
            end
        end
        % 深刻な異常があればスナップショットを保存（常時保存）
        if max(abs(obj.p(:)))>1e12 || any(isnan([obj.p(:); obj.v(:); obj.q(:); obj.P(:)]))
            outdir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Results'); if ~exist(outdir,'dir'), mkdir(outdir); end
            fname = fullfile(outdir, sprintf('state_pre_snapshot_%s_%d.mat', sensor_type, randi(1e9)));
            try
                snapshot.state.p = obj.p(:);
                snapshot.state.v = obj.v(:);
                snapshot.state.q = obj.q(:);
                snapshot.state.ba = obj.ba(:);
                snapshot.state.bg = obj.bg(:);
                snapshot.state.P = obj.P;
                snapshot.meta.dt = obj.dt;
                snapshot.meta.t = now;
                save(fname, 'snapshot');
            catch
            end
        end
    catch
    end

    state.p = obj.p(:);
    state.v = obj.v(:);
    state.q = obj.q(:);
    state.ba = obj.ba(:);
    state.bg = obj.bg(:);
    state.P = obj.P;

    % 呼び出し
    try
        % Debug: call and check for NaN in returned state
        new_state = mex_meukf_step_v2(state, sensor_data, mex_params);
        % Validate outputs
        fields_ok = isstruct(new_state) && isfield(new_state,'p') && isfield(new_state,'v') && isfield(new_state,'q') && isfield(new_state,'ba') && isfield(new_state,'bg') && isfield(new_state,'P');
        if ~fields_ok || any(isnan([new_state.p(:); new_state.v(:); new_state.q(:); new_state.ba(:); new_state.bg(:)])) || any(isnan(new_state.P(:)))
            try
                outdir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Results'); if ~exist(outdir,'dir'), mkdir(outdir); end
                fname = fullfile(outdir, sprintf('debug_mex_nan_%s_%d.mat', sensor_type, randi(1e9)));
                debug_dump.state = state;
                debug_dump.sensor_data = sensor_data;
                debug_dump.mex_params = mex_params;
                debug_dump.new_state = new_state;
                save(fname, 'debug_dump');
                fprintf('ESKF: do_cpp_update detected NaN in new_state — saved debug to %s\n', fname);
            catch
            end
            error('ESKF:do_cpp_update:NaN','C++ update produced NaN or invalid outputs for sensor %s', sensor_type);
        end

        obj.p = new_state.p;
        obj.v = new_state.v;
        obj.q = new_state.q;
        obj.ba = new_state.ba;
        obj.bg = new_state.bg;
        obj.P = new_state.P;

        % post-update trace
        try
            if strcmp(getenv('ENABLE_STATE_TRACE'),'1')
                outdir2 = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Results'); if ~exist(outdir2,'dir'), mkdir(outdir2); end
                fid2 = fopen(fullfile(outdir2,'state_trace.txt'),'a');
                if fid2 ~= -1
                    fprintf(fid2, '%s POST sensor=%s p_max=%g v_max=%g q_norm=%g P_max=%g\n', datestr(now,'yyyy-mm-dd HH:MM:SS.FFF'), sensor_type, max(abs(obj.p(:))), max(abs(obj.v(:))), norm(obj.q(:)), max(abs(obj.P(:))));
                    fclose(fid2);
                end
            end
        catch
        end
    catch ME
        warning('ESKF:do_cpp_update:Failed', 'C++ %s update failed: %s', sensor_type, ME.message);
        rethrow(ME);
    end
end

