function fft_analysis(T, cfg)
% fft_analysis - perform frequency analysis (spectrum) and plot
% T : table loaded from CSV
% cfg : configuration struct from fft_config

fs = cfg.fs;
nfft = cfg.nfft;
fmax = cfg.fmax;

% helper to extract columns
getcol = @(names) get_col_from_table(T, names);

sensors = {
    {'accel3_x','accel3_y','accel3_z'}, 'Accelerometer (body)';
    {'gyro3_x','gyro3_y','gyro3_z'}, 'Gyroscope';
    {'mag3_x','mag3_y','mag3_z'}, 'Magnetometer';
    {'gps_x','gps_y'}, 'GPS (x/y)';
    % baro removed from FFT analysis list
    {'meas_heading_x','meas_heading_y'}, 'Heading vector';
};

for i=1:size(sensors,1)
    cols = sensors{i,1}; label = sensors{i,2};
    data = getcol(cols);
    if all(isnan(data(:)))
        fprintf('Skipping %s (no columns found)\n', label);
        continue;
    end
    D = size(data,2);
    figure('Name',['FFT - ' label]);
    for d=1:D
        x = data(:,d); x(isnan(x)) = 0; x = x-mean(x);
        Y = fft(x, nfft);
        P2 = abs(Y/numel(x)); P1 = P2(1:nfft/2+1); P1(2:end-1) = 2*P1(2:end-1);
        f = fs*(0:(nfft/2))/nfft;
        subplot(D,1,d);
        plot(f, P1);
        xlabel('Frequency (Hz)'); ylabel('Amplitude'); title(sprintf('%s - axis %d', label, d));
        xlim([0, min(fmax, fs/2)]);
    end
end
end
