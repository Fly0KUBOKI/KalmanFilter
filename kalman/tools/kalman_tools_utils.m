classdef kalman_tools_utils
    % kalman_tools_utils  Common utilities for Phase3 analysis and diagnostics
    % 
    % Usage:
    %   base = kalman_tools_utils.get_project_root();
    %   T = kalman_tools_utils.read_sensor_data();
    %   times = kalman_tools_utils.parse_top_diffs(max_count);
    
    methods (Static)
        %% Path utilities
        function base = get_project_root()
            % Get KalmanFilter project root (parent of 'kalman' folder)
            tools_path = fileparts(mfilename('fullpath'));
            kalman_path = fileparts(tools_path);
            base = fileparts(kalman_path);
        end
        
        function results_dir = get_results_dir(kalman_subfolder)
            % Get Results directory (kalman-level or parent-level)
            if nargin < 1 || isempty(kalman_subfolder)
                kalman_subfolder = true;
            end
            base = kalman_tools_utils.get_project_root();
            if kalman_subfolder
                results_dir = fullfile(base, 'kalman', 'Results');
            else
                results_dir = fullfile(base, 'Results');
            end
        end
        
        function gen_dir = get_gen_dir()
            % Get GenerateData directory
            base = kalman_tools_utils.get_project_root();
            gen_dir = fullfile(base, 'kalman', 'GenerateData');
        end
        
        %% File I/O
        function T = read_sensor_data()
            % Load sensor_data.csv and return table
            gen_dir = kalman_tools_utils.get_gen_dir();
            sensor_csv = fullfile(gen_dir, 'sensor_data.csv');
            if ~exist(sensor_csv, 'file')
                error('sensor_data.csv not found: %s', sensor_csv);
            end
            T = readtable(sensor_csv);
        end
        
        function truth = read_truth_data()
            % Load truth_data.csv and return table
            gen_dir = kalman_tools_utils.get_gen_dir();
            truth_csv = fullfile(gen_dir, 'truth_data.csv');
            if ~exist(truth_csv, 'file')
                error('truth_data.csv not found: %s', truth_csv);
            end
            truth = readtable(truth_csv);
        end
        
        function times = parse_top_diffs(max_count, results_dir_override)
            % Parse times from top_diffs.txt
            % Usage: times = parse_top_diffs() or parse_top_diffs(50)
            if nargin < 1 || isempty(max_count)
                max_count = 50;
            end
            if nargin < 2 || isempty(results_dir_override)
                results_dir = kalman_tools_utils.get_results_dir(true);
            else
                results_dir = results_dir_override;
            end
            
            topfile = fullfile(results_dir, 'top_diffs.txt');
            if ~exist(topfile, 'file')
                error('top_diffs.txt not found: %s', topfile);
            end
            
            fid = fopen(topfile, 'r');
            lines = {};
            while ~feof(fid)
                lines{end+1} = strtrim(fgetl(fid));
            end
            fclose(fid);
            
            times = [];
            for i = 2:min(length(lines), max_count + 1)
                if isempty(lines{i}), continue; end
                toks = strsplit(lines{i}, ',');
                t = str2double(toks{2});
                if ~isnan(t), times(end+1) = t; end
            end
        end
        
        %% Data extraction helpers
        function main = pick_main_var(S, vars)
            % Pick main variable from loaded struct (prefer 'record'/'rec'/'r', else first non-empty)
            main = [];
            for p = {'record', 'rec', 'r'}
                if ismember(p{1}, vars)
                    main = S.(p{1});
                    return;
                end
            end
            for k = 1:numel(vars)
                val = S.(vars{k});
                if ~isempty(val)
                    main = val;
                    return;
                end
            end
        end
        
        function v = safe_get(s, name)
            % Safely get field from struct (handles nested structures)
            v = [];
            if isempty(s), return; end
            if isstruct(s) && isfield(s, name)
                v = s.(name);
            else
                if isstruct(s)
                    fn = fieldnames(s);
                    for k = 1:numel(fn)
                        f = s.(fn{k});
                        if isstruct(f) && isfield(f, name)
                            v = f.(name);
                            return;
                        end
                    end
                end
            end
        end
        
        %% Comparison helpers
        function [idx_time, idx_sample] = find_nearest_samples(times, time_vec)
            % For each time in times, find nearest sample index in time_vec
            % Returns: idx_time (input time index), idx_sample (matched sample)
            idx_time = 1:numel(times);
            idx_sample = zeros(size(times));
            for i = 1:numel(times)
                [~, idx_sample(i)] = min(abs(time_vec - times(i)));
            end
        end
        
        function rmse_val = compute_rmse(est, truth_col)
            % Compute RMS error between estimate and truth column
            rmse_val = rms(est - truth_col);
        end
        
        function delta_fro = matrix_delta_fro(M1, M2)
            % Compute Frobenius norm of M2 - M1
            if isempty(M1) || isempty(M2)
                delta_fro = [];
            else
                delta_fro = norm(M2 - M1, 'fro');
            end
        end
    end
end
