function cfg = fft_config()
% FFT configuration for analysis and lowpass filtering
% Returned as struct cfg with fields:
%  .csvfile        - path to CSV (default: sim_data.csv in pwd)
%  .dt             - sampling interval in seconds
%  .fs             - sampling frequency
%  .mode           - 'freq' or 'lowpass'
%  .fmax           - frequency display upper limit (Hz)
%  .nfft           - FFT length
%  .lowpass_cutoff - cutoff for lowpass mode (Hz)

cfg = struct();

% CSV location: prefer config_params default (GenerateData location) then fallback to pwd
try
    p = config_params();
    if isfield(p,'data') && isfield(p.data,'file') && ~isempty(p.data.file)
        cfg.csvfile = p.data.file;
    else
        cfg.csvfile = fullfile(pwd, 'sim_data.csv');
    end
catch
    cfg.csvfile = fullfile(pwd, 'sim_data.csv');
end

% dt: try to read from config_params(), otherwise fallback
try
    p = config_params(); cfg.dt = p.dt;
catch
    cfg.dt = 0.1;
end
cfg.fs = 1/cfg.dt;

% mode and FFT params
cfg.mode = 'lowpass'; % 'freq' or 'lowpass'
cfg.fmax = min(10, cfg.fs/2); % display up to 10 Hz by default
cfg.nfft = 8192; % FFT length

% lowpass cutoff (used only when mode == 'lowpass')
cfg.lowpass_cutoff = 0.1; % Hz

end
