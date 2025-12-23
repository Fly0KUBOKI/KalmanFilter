function results = mex_run_filter(eskf, obs)
% MEX_RUN_FILTER  MATLAB fallback for Phase 8
% If a compiled MEX `mex_run_filter` is not available, this MATLAB
% implementation performs the same main-loop semantics so the project
% can be tested before a full C++ implementation is completed.

n_samples = numel(obs.time);
params = config_params();
static_samples = floor(params.static_time / eskf.dt);

results.time = obs.time(:)';
results.p = zeros(3, n_samples);
results.v = zeros(3, n_samples);
results.euler = zeros(3, n_samples);
results.ba = zeros(3, n_samples);
results.bg = zeros(3, n_samples);
results.innov_norm = zeros(1, n_samples);
results.maha_dist = zeros(1, n_samples);

ENABLE_SAVE_TRACES = false;

for k = 1:n_samples
    a = [obs.ax(k); obs.ay(k); obs.az(k)];
    w = deg2rad([obs.wx(k); obs.wy(k); obs.wz(k)]);

    if k > static_samples
        eskf.predict(a, w);
        if eskf.zupt('check', a, w); eskf.zupt('update'); end

        if mod(k, eskf.freq_accel) == 0
            eskf.sensor_updates('accel', a, k);
        end
        if mod(k, eskf.freq_mag) == 0
            eskf.sensor_updates('mag', [obs.mx(k); obs.my(k); obs.mz(k)], k);
        end
        if mod(k, eskf.freq_baro) == 0
            eskf.sensor_updates('baro', obs.pressure(k), k);
        end
        if mod(k, eskf.freq_gps) == 0 && ~isnan(obs.lat(k))
            eskf.sensor_updates('gps', obs.lat(k), obs.lon(k), obs.alt(k), k);
        end
    end

    eskf.reset('check', obs, k);

    results.p(:,k) = eskf.p;
    results.v(:,k) = eskf.v;
    results.euler(:,k) = eskf.utils('get_euler');
    results.ba(:,k) = eskf.ba;
    results.bg(:,k) = eskf.bg;
    results.innov_norm(k) = 0;
    results.maha_dist(k) = 0;
end

end
