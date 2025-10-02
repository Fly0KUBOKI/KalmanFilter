function run_fft_from_csv()
% run_fft_from_csv - simple dispatcher for FFT tools
% Loads config from fft_config(), reads the CSV, and calls the selected
% processing mode ('freq' or 'lowpass').
%
% Usage: run_fft_from_csv()

cfg = fft_config();
if ~isfield(cfg, 'csvfile') || ~exist(cfg.csvfile, 'file')
    error('CSV file not found: %s', getfield(cfg,'csvfile'));
end
T = readtable(cfg.csvfile);

if ~isfield(cfg, 'mode')
    error('fft_config must set cfg.mode to ''freq'' or ''lowpass''.');
end

% ---------------------------------------------------------------------
% Define variables to show after processing (edit here)
% Each entry should be a column name present in the CSV (or a name
% that get_col_from_table can resolve). You can add multiple entries.
plot_def.columns = {'gyro3_x','gyro3_y','gyro3_z','baro'};   % e.g. {'accel3_x','accel3_y'}
plot_def.labels  = {'Gyro X','Gyro Y','Gyro Z','Baro'};    % optional human-readable labels
% ---------------------------------------------------------------------

switch lower(cfg.mode)
    case 'freq'
        fft_analysis(T, cfg);
    case 'lowpass'
        T_out = fft_lowpass_filter(T, cfg);
        % optional example plots: use the define block above
        for k = 1:numel(plot_def.columns)
            col = plot_def.columns{k};
            if numel(plot_def.labels) >= k && ~isempty(plot_def.labels{k})
                lab = plot_def.labels{k};
            else
                lab = col;
            end
            if ismember(col, T.Properties.VariableNames) && ismember(col, T_out.Properties.VariableNames)
                figure('Name', sprintf('%s - original vs filtered', lab));
                t = (0:height(T)-1) * cfg.dt;
                plot(t, T.(col), '-k'); hold on; plot(t, T_out.(col), '-r'); hold off;
                legend({'original','filtered'}); xlabel('Time (s)'); ylabel(lab);
            end
        end
    otherwise
        error('Unknown cfg.mode: %s', cfg.mode);
end

fprintf('FFT wrapper finished.\n');
end