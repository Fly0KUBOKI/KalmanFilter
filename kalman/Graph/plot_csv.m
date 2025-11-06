function varargout = plot_csv(filePath, mode)
    % PLOT_CSVの可視化（モード切替）
    % modes: 'time' (default), 'pos_xy', 'vel_xy', 'att_xy'

    if nargin<1 || isempty(filePath)
        filePath = fullfile(fileparts(mfilename('fullpath')),'..','Results','estimation.csv');
    end
    if nargin<2 || isempty(mode)
        mode = 'pos';
    end

    T = readtable(filePath);

    % Load truth data (no error handling as requested)
    truthFile = fullfile(fileparts(mfilename('fullpath')),'..','GenerateData','truth_data.csv');
    TR = readtable(truthFile);

    switch mode
        case 'time'
            time = T.time;
            fh = figure;
            subplot(3,1,1);
            plot(time, T.px, time, T.py, time, T.pz);
            legend('px','py','pz'); title('Position [m]'); grid on;

            % Overlay truth (dashed). Interpolate truth to estimation time.
            hold on
            tx = interp1(TR.time, TR.x, time, 'linear', NaN);
            ty = interp1(TR.time, TR.y, time, 'linear', NaN);
            tz = interp1(TR.time, TR.z, time, 'linear', NaN);
            plot(time, tx, '--r'); plot(time, ty, '--g'); plot(time, tz, '--k');
            legend('px','py','pz','px_truth','py_truth','pz_truth');

            subplot(3,1,2);
            plot(time, T.vx, time, T.vy, time, T.vz);
            legend('vx','vy','vz'); title('Velocity [m/s]'); grid on;

            hold on
            tvx = interp1(TR.time, TR.vx, time, 'linear', NaN);
            tvy = interp1(TR.time, TR.vy, time, 'linear', NaN);
            tvz = interp1(TR.time, TR.vz, time, 'linear', NaN);
            plot(time, tvx, '--r'); plot(time, tvy, '--g'); plot(time, tvz, '--k');
            legend('vx','vy','vz','vx_truth','vy_truth','vz_truth');

            subplot(3,1,3);
            plot(time, T.roll, time, T.pitch, time, T.yaw);
            legend('roll','pitch','yaw'); title('Attitude [rad]'); grid on;

            hold on
            tr_roll = interp1(TR.time, TR.roll, time, 'linear', NaN);
            tr_pitch = interp1(TR.time, TR.pitch, time, 'linear', NaN);
            tr_yaw = interp1(TR.time, TR.yaw, time, 'linear', NaN);
            plot(time, tr_roll, '--r'); plot(time, tr_pitch, '--g'); plot(time, tr_yaw, '--k');
            legend('roll','pitch','yaw','roll_truth','pitch_truth','yaw_truth');

        case 'pos'
            fh = figure;
            X = T.px; Y = T.py;
            
            % Load GPS observation data
            sensorFile = fullfile(fileparts(mfilename('fullpath')),'..','GenerateData','sensor_data.csv');
            try
                SD = readtable(sensorFile);
                % Convert GPS lat/lon to local coordinates (simplified conversion)
                % Assuming the origin is at the first GPS point
                if ~isempty(SD.gps_lat) && ~isempty(SD.gps_lon)
                    lat0 = SD.gps_lat(1);
                    lon0 = SD.gps_lon(1);
                    % Approximate conversion: 1 degree lat ≈ 111320 m, lon depends on latitude
                    R_earth = 6378137; % Earth radius in meters
                    gps_x = (SD.gps_lon - lon0) * cos(lat0 * pi/180) * pi/180 * R_earth;
                    gps_y = (SD.gps_lat - lat0) * pi/180 * R_earth;
                else
                    gps_x = [];
                    gps_y = [];
                end
            catch
                warning('Could not load GPS observation data');
                gps_x = [];
                gps_y = [];
            end
            
            % 固定された軸範囲（xyともに -15 から 15）
            axis([-15 15 -15 15]);
            hold on; grid on; axis equal;
            hpath = plot(NaN, NaN, '-b');
            hcur = plot(NaN, NaN, 'ro');
            plot(X(1), Y(1), 'go'); % start
            plot(X(end), Y(end), 'ko'); % end (keep end marker)
            % Overlay truth trajectory (dashed)
            plot(TR.x, TR.y, '--', 'Color', [0.5 0.5 0.5]);
            
            % Initialize GPS plot handle (will be updated during animation)
            hgps = plot(NaN, NaN, 'ro', 'MarkerSize', 2, 'DisplayName', 'GPS observations');
            
            xlabel('x [m]'); ylabel('y [m]');
            for i = 1:numel(X)
                set(hpath, 'XData', X(1:i), 'YData', Y(1:i));
                set(hcur, 'XData', X(i), 'YData', Y(i));
                
                % Update GPS observations (every 40th point up to current time)
                if ~isempty(gps_x)
                    current_gps_indices = 1:40:min(i, length(gps_x));
                    if ~isempty(current_gps_indices)
                        set(hgps, 'XData', gps_x(current_gps_indices), 'YData', gps_y(current_gps_indices));
                    end
                end
                
                drawnow limitrate
            end

        case 'vel'
            % ベクトルは原点 (0,0) から表示する
            U = T.vx; V = T.vy;
            fh = figure;
            maxmag = max(sqrt(U.^2 + V.^2));
            if isempty(maxmag) || maxmag==0, maxmag = 1; end
            lim = maxmag * 1.1;
            hq = quiver(0, 0, U(1), V(1), 0); axis equal; grid on;
            axis([-lim lim -lim lim]);
            for i = 1:numel(U)
                set(hq, 'XData', 0, 'YData', 0, 'UData', U(i), 'VData', V(i));
                drawnow limitrate
            end

        case 'att'
            % Use yaw as heading in x-y plane (yaw expected in radians)
            % ベクトルは原点 (0,0) から表示する（yaw は CSV が度ならラジアンに変換）
            Yaw = T.yaw * pi/180; % CSV の角度（deg）を rad に変換
            U = cos(Yaw); V = sin(Yaw);
            fh = figure;
            lim = 1.1; % cos/sin の範囲に合わせる
            hq = quiver(0, 0, U(1), V(1), 0); axis equal; grid on;
            axis([-lim lim -lim lim]);
            for i = 1:numel(U)
                set(hq, 'XData', 0, 'YData', 0, 'UData', U(i), 'VData', V(i));
                drawnow limitrate
            end
        
        otherwise
            error('Unknown mode: %s', mode);
    end

    if nargout>0
        varargout{1} = fh;
    end
end
