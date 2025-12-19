function dump_records_for_top_diffs()
% Read top_diffs.txt and for each time find nearest sample index then dump
% record_before_* and record_after_* MAT files if present.
results_dir = kalman_tools_utils.get_results_dir(true);

% Parse times from top_diffs.txt
times = kalman_tools_utils.parse_top_diffs(50);

% Read sensor times
T = kalman_tools_utils.read_sensor_data();
time_vec = T{:,1};

sensor_names = {'gps','baro','mag','accel'};

for tt = times
    % find nearest sample index
    [~, idx] = min(abs(time_vec - tt));
    outtxt = fullfile(results_dir, sprintf('dump_records_time_%.4f_idx_%d.txt', tt, idx));
    fid = fopen(outtxt,'w');
    fprintf(fid,'Dump for time %.6f, nearest sample idx=%d, sim_time=%f\n\n', tt, idx, time_vec(idx));
    for s = 1:length(sensor_names)
        sn = sensor_names{s};
        before_name = fullfile(results_dir, sprintf('record_before_%s_%d.mat', sn, idx));
        after_name  = fullfile(results_dir, sprintf('record_after_%s_%d.mat', sn, idx));
        fprintf(fid,'--- Sensor: %s ---\n', sn);
        if exist(before_name,'file')
            try
                SB = load(before_name);
                fn = fieldnames(SB);
                fprintf(fid,'Before file: %s\n', before_name);
                for k=1:length(fn)
                    fprintf(fid,'  %s\n', fn{k});
                end
                % try to extract state, P if present
                if isfield(SB,'eskf_state')
                    st = SB.eskf_state;
                    try fprintf(fid,'  eskf_state.p = %g %g %g\n', st.p); catch end
                    try fprintf(fid,'  P11-33:\n'); for r=1:3, fprintf(fid,'   %g %g %g\n', st.P(r,1:3)); end; catch end
                end
            catch e
                fprintf(fid,'  Error loading before: %s\n', e.message);
            end
        else
            fprintf(fid,'  Before file not found: %s\n', before_name);
        end
        if exist(after_name,'file')
            try
                SA = load(after_name);
                fn = fieldnames(SA);
                fprintf(fid,'After file: %s\n', after_name);
                for k=1:length(fn)
                    fprintf(fid,'  %s\n', fn{k});
                end
                if isfield(SA,'new_state')
                    ns = SA.new_state;
                    try fprintf(fid,'  new_state.p = %g %g %g\n', ns.p); catch end
                    try fprintf(fid,'  new_state.P11-33:\n'); for r=1:3, fprintf(fid,'   %g %g %g\n', ns.P(r,1:3)); end; catch end
                end
            catch e
                fprintf(fid,'  Error loading after: %s\n', e.message);
            end
        else
            fprintf(fid,'  After file not found: %s\n', after_name);
        end
        fprintf(fid,'\n');
    end
    fclose(fid);
end
fprintf('Dumped records for %d times to %s\n', length(times), results_dir);
end
