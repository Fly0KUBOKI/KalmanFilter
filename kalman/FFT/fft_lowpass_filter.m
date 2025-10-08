function T_out = fft_lowpass_filter(T, cfg)
% fft_lowpass_filter - apply FFT-domain ideal low-pass to table signals
% T : input table
% cfg : config struct from fft_config
% Returns T_out: copy of T with filtered columns for the signals that exist

fs = cfg.fs; nfft = cfg.nfft; cutoff = min(cfg.lowpass_cutoff, fs/2);
T_out = T;

sensors = {
    {'accel3_x','accel3_y','accel3_z'};
    {'gyro3_x','gyro3_y','gyro3_z'};
    {'mag3_x','mag3_y','mag3_z'};
    {'gps_x','gps_y'};
    {'baro'};
    {'meas_heading_x','meas_heading_y'};
};

for i=1:numel(sensors)
    cols = sensors{i};
    % try to extract
    data = get_col_from_table(T, cols);
    if all(isnan(data(:)))
        continue;
    end
    D = size(data,2);
    for d=1:D
        x = data(:,d); x(isnan(x)) = 0; x = x - mean(x);
        Y = fft(x, nfft);
        f_full = fs*(0:(nfft-1))/nfft;
        mask = (f_full <= cutoff) | (f_full >= fs - cutoff);
        Y(~mask) = 0;
        xf = real(ifft(Y, nfft)); xf = xf(1:numel(x));
        % write back to table column if column exists
        colname = cols{d};
        if ismember(colname, T.Properties.VariableNames)
            T_out.(colname) = xf;
        else
            % try alternates
            alt1 = ['meas_' colname];
            alt2 = [colname '_meas'];
            if ismember(alt1, T.Properties.VariableNames)
                T_out.(alt1) = xf;
            elseif ismember(alt2, T.Properties.VariableNames)
                T_out.(alt2) = xf;
            end
        end
    end
end
end
