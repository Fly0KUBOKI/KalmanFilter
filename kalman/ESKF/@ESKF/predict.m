function predict(obj, a_meas, w_meas)
    % 予測ステップ (C++実装: mex_meukf_step_v2)
    
    % NaN check
    if any(isnan(obj.p)) || any(isnan(obj.v)) || any(isnan(obj.q)) || any(isnan(obj.P(:)))
        warning('ESKF:predict:NaN', 'NaN detected before predict');
        return;
    end
    
    % ジャイロフィルタ適用 (MATLAB側で実施)
    if isprop(obj, 'enable_gyro_filter') && ~isempty(obj.enable_gyro_filter) && obj.enable_gyro_filter && ...
       isfield(obj.sensor_filters, 'gyro') && ~isempty(obj.sensor_filters.gyro)
        w_expected = obj.sensor_filters.gyro.w_filtered;
        [w_filtered, w_is_outlier, ~] = obj.sensor_filters.gyro.apply(w_meas, w_expected);
        if w_is_outlier
            w_meas = w_expected;
        else
            if obj.enable_yaw_raw_gyro
                w_filtered(3) = w_meas(3);
            end
            w_meas = w_filtered;
        end
    end
    
    % 加速度フィルタ適用 (MATLAB側で実施)
    if ~isempty(obj.accel_filter)
        a_expected = obj.accel_filter.a_filtered;
        if norm(a_expected) < 1e-3
            a_expected = a_meas;
        end
        [a_filtered, is_outlier] = obj.accel_filter.filter(a_meas, a_expected);
        a_for_vel = a_filtered;
    else
        a_for_vel = a_meas;
    end
    
    % Adaptive Q (MATLAB側で計算)
    Q_adapted = obj.Q;
    if obj.adaptive_q_enabled
        a_norm = norm(a_meas);
        gravity_error = abs(a_norm - 9.81);
        accel_scale = 1.0 + (gravity_error / 3.0);
        w_norm = norm(w_meas);
        gyro_scale = 1.0 + (w_norm / deg2rad(15.0));
        q_scale = max(accel_scale, gyro_scale);
        q_scale = min(q_scale, 5.0);
        Q_adapted = obj.Q_nominal * q_scale;
    end
    
    % C++ MEX用パラメータ準備
    dt2 = obj.dt^2;
    noise_accel = diag(Q_adapted(4:6, 4:6)) / dt2;
    noise_gyro = diag(Q_adapted(7:9, 7:9)) / dt2;
    noise_ba = diag(Q_adapted(10:12, 10:12)) / obj.dt;
    noise_bg = diag(Q_adapted(13:15, 13:15)) / obj.dt;
    
    % センサー構造体
    sensor_data.accel = a_for_vel;
    sensor_data.gyro = w_meas;
    sensor_data.mag = zeros(3,1);
    sensor_data.gps_pos = zeros(3,1);
    sensor_data.alt_baro = 0;
    sensor_data.dt = obj.dt;
    
    % 更新フラグ (予測のみ)
    sensor_data.update_accel = false;
    sensor_data.update_gyro = false;
    sensor_data.update_mag = false;
    sensor_data.update_gps = false;
    sensor_data.update_baro = false;
    
    % パラメータ構造体
    mex_params.g = obj.g;
    mex_params.mag_ref = [50; 0; 0]; % Dummy
    mex_params.noise_accel = noise_accel;
    mex_params.noise_gyro = noise_gyro;
    mex_params.noise_ba = noise_ba;
    mex_params.noise_bg = noise_bg;
    mex_params.noise_mag = zeros(3,1);
    mex_params.noise_gps = zeros(3,1);
    mex_params.noise_baro = 0;
    mex_params.alpha = 1e-3;
    mex_params.beta = 2;
    mex_params.kappa = 0;
    
    % 状態構造体
    state_in.p = obj.p;
    state_in.v = obj.v;
    state_in.q = obj.q;
    state_in.ba = obj.ba;
    state_in.bg = obj.bg;
    state_in.P = obj.P;
    % TRACE_SAMPLE: save pre-predict state if TRACE_SAMPLE env var set
    try
        trace_sample_env = getenv('TRACE_SAMPLE');
        trace_sample_num = str2double(trace_sample_env);
        if ~isnan(trace_sample_num)
            outdir_trace = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Results'); if ~exist(outdir_trace,'dir'), mkdir(outdir_trace); end
            fname_pre = fullfile(outdir_trace, sprintf('predict_trace_%d_pre.mat', trace_sample_num));
            try
                trace_pre.state_in = state_in;
                trace_pre.mex_params = mex_params;
                trace_pre.sensor_data = sensor_data;
                trace_pre.objP_pre = obj.P;
                save(fname_pre, 'trace_pre');
            catch
            end
        end
    catch
    end
    % 上流ログ (predict) — verbose は環境変数で制御
    try
        if strcmp(getenv('ENABLE_STATE_TRACE'),'1')
            outdir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Results'); if ~exist(outdir,'dir'), mkdir(outdir); end
            fid = fopen(fullfile(outdir,'state_trace.txt'),'a');
            if fid~=-1
                fprintf(fid, '%s sensor=predict dt=%g p_max=%g v_max=%g q_norm=%g P_max=%g\n', datestr(now,'yyyy-mm-dd HH:MM:SS.FFF'), obj.dt, max(abs(obj.p(:))), max(abs(obj.v(:))), norm(obj.q(:)), max(abs(obj.P(:))));
                fclose(fid);
            end
        end
        if max(abs(obj.p(:)))>1e12 || any(isnan([obj.p(:); obj.v(:); obj.q(:); obj.P(:)]))
            outdir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Results'); if ~exist(outdir,'dir'), mkdir(outdir); end
            fname = fullfile(outdir, sprintf('state_pre_snapshot_predict_%d.mat', randi(1e9)));
            try
                snapshot.state.p = obj.p(:);
                snapshot.state.v = obj.v(:);
                snapshot.state.q = obj.q(:);
                snapshot.state.P = obj.P;
                snapshot.meta.dt = obj.dt;
                snapshot.meta.t = now;
                save(fname, 'snapshot');
            catch
            end
        end
    catch
    end

    % C++ MEX呼び出し
    try
        state_out = mex_meukf_step_v2(state_in, sensor_data, mex_params);
        
        % 状態更新
        obj.p = state_out.p;
        obj.v = state_out.v;
        obj.q = state_out.q;
        obj.ba = state_out.ba;
        obj.bg = state_out.bg;
        obj.P = state_out.P;

        % post-update trace for predict (env controlled)
        try
            if strcmp(getenv('ENABLE_STATE_TRACE'),'1')
                outdir2 = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Results'); if ~exist(outdir2,'dir'), mkdir(outdir2); end
                fid2 = fopen(fullfile(outdir2,'state_trace.txt'),'a');
                if fid2~=-1
                    fprintf(fid2, '%s POST sensor=predict dt=%g p_max=%g v_max=%g q_norm=%g P_max=%g\n', datestr(now,'yyyy-mm-dd HH:MM:SS.FFF'), obj.dt, max(abs(obj.p(:))), max(abs(obj.v(:))), norm(obj.q(:)), max(abs(obj.P(:))));
                    fclose(fid2);
                end
            end
        catch
        end
        
    catch ME
        warning('ESKF:predict:MEXFailed', 'C++ Predict failed: %s', ME.message);
        return;
    end

    % TRACE_SAMPLE: save post-predict state if TRACE_SAMPLE env var set
    try
        if exist('trace_sample_num','var') && ~isnan(trace_sample_num)
            outdir_trace2 = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Results'); if ~exist(outdir_trace2,'dir'), mkdir(outdir_trace2); end
            fname_post = fullfile(outdir_trace2, sprintf('predict_trace_%d_post.mat', trace_sample_num));
            try
                trace_post.state_out = state_out;
                trace_post.mex_params = mex_params;
                trace_post.sensor_data = sensor_data;
                trace_post.objP_post = obj.P;
                save(fname_post, 'trace_post');
            catch
            end
        end
    catch
    end

    % 角速度を保存
    obj.w_body = w_meas;
    obj.quaternion_norm = norm(obj.q);
    
    % Z軸加速度積分
    if obj.enable_accel_z_integration
        R = mex_quaternion_lib('to_rotation_matrix', obj.q);
        a_ned = R * a_for_vel - [0; 0; obj.g(3)];
        az_excess = a_ned(3);
        if abs(az_excess) > obj.accel_z_threshold
            obj.v(3) = obj.v(3) * (1.0 - obj.accel_z_damping) + az_excess * obj.dt;
        end
    end
    
    % XY速度減衰
    if ~isempty(obj.velocity_damping) && obj.velocity_damping > 0
        obj.v(1:2) = obj.v(1:2) * (1.0 - obj.velocity_damping * obj.dt);
    end
    
    % 共分散正則化と制限
    obj.P = obj.divergence_guard.regularize_covariance(obj.P);
    max_var = [100^2*ones(3,1); 20^2*ones(3,1); (deg2rad(45))^2*ones(3,1); 0.1*ones(3,1); 0.01*ones(3,1)];
    for i = 1:15
        if obj.P(i,i) > max_var(i)
            factor = sqrt(max_var(i) / obj.P(i,i));
            obj.P(i,:) = obj.P(i,:) * factor;
            obj.P(:,i) = obj.P(:,i) * factor;
            obj.P(i,i) = max_var(i);
        end
    end
    
    [obj.v, obj.P, ~] = obj.divergence_guard.check_and_clip_velocity(obj.v, obj.P, 4:6);
end
