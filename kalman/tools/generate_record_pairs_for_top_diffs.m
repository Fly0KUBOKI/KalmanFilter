function generate_record_pairs_for_top_diffs(max_count)
% generate_record_pairs_for_top_diffs Create record_before_/record_after_ MAT files
% for the top diffs listed in Results/top_diffs.txt.
% Usage: generate_record_pairs_for_top_diffs()  % uses up to 50 times
%        generate_record_pairs_for_top_diffs(10)

if nargin<1 || isempty(max_count), max_count = 50; end
results_dir = kalman_tools_utils.get_results_dir(true);
proj_root = kalman_tools_utils.get_project_root();

% Parse times from top_diffs.txt
times = kalman_tools_utils.parse_top_diffs(max_count);
if isempty(times), warning('No times parsed from top_diffs'); return; end

% Read sensor times
T = kalman_tools_utils.read_sensor_data();
time_vec = T{:,1};

indices = zeros(size(times));
for i=1:numel(times)
    [~, idx] = min(abs(time_vec - times(i))); indices(i) = idx;
end
indices = unique(indices);
fprintf('Generating record pairs for %d indices\n', numel(indices));

% load observations and configs
addpath(genpath(proj_root)); % ensure KF/ESKF on path
% build obs struct directly from table T (sensor_data.csv)
obs.time = time_vec(:);
cols = T.Properties.VariableNames;
if ismember('ax', cols), obs.ax = T.ax; else obs.ax = zeros(size(obs.time)); end
if ismember('ay', cols), obs.ay = T.ay; else obs.ay = zeros(size(obs.time)); end
if ismember('az', cols), obs.az = T.az; else obs.az = zeros(size(obs.time)); end
if ismember('wx', cols), obs.wx = T.wx; else obs.wx = zeros(size(obs.time)); end
if ismember('wy', cols), obs.wy = T.wy; else obs.wy = zeros(size(obs.time)); end
if ismember('wz', cols), obs.wz = T.wz; else obs.wz = zeros(size(obs.time)); end
if ismember('mx', cols), obs.mx = T.mx; else obs.mx = zeros(size(obs.time)); end
if ismember('my', cols), obs.my = T.my; else obs.my = zeros(size(obs.time)); end
if ismember('mz', cols), obs.mz = T.mz; else obs.mz = zeros(size(obs.time)); end
if ismember('pressure', cols), obs.pressure = T.pressure; else obs.pressure = zeros(size(obs.time)); end
if ismember('lat', cols), obs.lat = T.lat; else obs.lat = nan(size(obs.time)); end
if ismember('lon', cols), obs.lon = T.lon; else obs.lon = nan(size(obs.time)); end
if ismember('alt', cols), obs.alt = T.alt; else obs.alt = nan(size(obs.time)); end

params = config_params(); dt = mean(diff(obs.time));
eskf = ESKF(obs, params.static_time, dt);

outdir = results_dir; if ~exist(outdir,'dir'), mkdir(outdir); end
idx_set = containers.Map('KeyType','double','ValueType','logical');
for i=1:numel(indices), idx_set(indices(i)) = true; end

n_samples = numel(obs.time);
for k=1:n_samples
    % predict
    a = [obs.ax(k); obs.ay(k); obs.az(k)];
    w = deg2rad([obs.wx(k); obs.wy(k); obs.wz(k)]);
    eskf.predict(a, w);

    if k > floor(params.static_time / eskf.dt)
        % accel
        if mod(k, eskf.freq_accel) == 0
            if isKey(idx_set, k)
                fname = fullfile(outdir, sprintf('record_before_accel_%d.mat', k));
                try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
            end
            eskf.sensor_updates('accel', a, k);
            if isKey(idx_set, k)
                fname = fullfile(outdir, sprintf('record_after_accel_%d.mat', k));
                try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
            end
        end

        % mag
        if mod(k, eskf.freq_mag) == 0
            if isKey(idx_set, k)
                fname = fullfile(outdir, sprintf('record_before_mag_%d.mat', k));
                try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
            end
            eskf.sensor_updates('mag', [obs.mx(k); obs.my(k); obs.mz(k)], k);
            if isKey(idx_set, k)
                fname = fullfile(outdir, sprintf('record_after_mag_%d.mat', k));
                try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
            end
        end

        % baro
        if mod(k, eskf.freq_baro) == 0
            if isKey(idx_set, k)
                fname = fullfile(outdir, sprintf('record_before_baro_%d.mat', k));
                try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
            end
            eskf.sensor_updates('baro', obs.pressure(k), k);
            if isKey(idx_set, k)
                fname = fullfile(outdir, sprintf('record_after_baro_%d.mat', k));
                try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
            end
        end

        % gps
        if mod(k, eskf.freq_gps) == 0 && ~isnan(obs.lat(k))
            if isKey(idx_set, k)
                fname = fullfile(outdir, sprintf('record_before_gps_%d.mat', k));
                try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
            end
            eskf.sensor_updates('gps', obs.lat(k), obs.lon(k), obs.alt(k), k);
            if isKey(idx_set, k)
                fname = fullfile(outdir, sprintf('record_after_gps_%d.mat', k));
                try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
            end
        end
    end
    eskf.reset('check', obs, k);
end

fprintf('Saved record_before/after pairs for %d indices to %s\n', numel(indices), outdir);
end
