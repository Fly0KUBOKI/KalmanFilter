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
            % 時系列比較をそれぞれ独立した figure に分ける
            time = T.time;

            % Position (px, py, pz)
            fh_pos = figure('Name', 'Position [m]');
            plot(time, T.px, '-','Color',[0 0 1],'LineWidth',1.5); hold on; % x: blue
            plot(time, T.py, '-','Color',[1 0 0],'LineWidth',1.5);           % y: red
            plot(time, T.pz, '-','Color',[1 0.75 0],'LineWidth',1.5);        % z: yellow-ish
            % Overlay truth (dashed) with same colors
            tx = interp1(TR.time, TR.x, time, 'linear', NaN);
            ty = interp1(TR.time, TR.y, time, 'linear', NaN);
            tz = interp1(TR.time, TR.z, time, 'linear', NaN);
            plot(time, tx, '--','Color',[0 0 1],'LineWidth',1.2);
            plot(time, ty, '--','Color',[1 0 0],'LineWidth',1.2);
            plot(time, tz, '--','Color',[1 0.75 0],'LineWidth',1.2);
            hold off; grid on; xlabel('時刻 [s]'); ylabel('Position [m]');
            legend('px','py','pz','px\_truth','py\_truth','pz\_truth','Location','best');

            % Velocity (vx, vy, vz)
            fh_vel = figure('Name', 'Velocity [m/s]');
            plot(time, T.vx, '-','Color',[0 0 1],'LineWidth',1.5); hold on; % x: blue
            plot(time, T.vy, '-','Color',[1 0 0],'LineWidth',1.5);           % y: red
            plot(time, T.vz, '-','Color',[1 0.75 0],'LineWidth',1.5);        % z: yellow-ish
            tvx = interp1(TR.time, TR.vx, time, 'linear', NaN);
            tvy = interp1(TR.time, TR.vy, time, 'linear', NaN);
            tvz = interp1(TR.time, TR.vz, time, 'linear', NaN);
            plot(time, tvx, '--','Color',[0 0 1],'LineWidth',1.2);
            plot(time, tvy, '--','Color',[1 0 0],'LineWidth',1.2);
            plot(time, tvz, '--','Color',[1 0.75 0],'LineWidth',1.2);
            hold off; grid on; xlabel('時刻 [s]'); ylabel('Velocity [m/s]');
            legend('vx','vy','vz','vx\_truth','vy\_truth','vz\_truth','Location','best');

            % Attitude: roll と pitch を専用 figure に表示（推定: 実線、真値: 点線）
            tr_roll = interp1(TR.time, TR.roll, time, 'linear', NaN);
            tr_pitch = interp1(TR.time, TR.pitch, time, 'linear', NaN);
            tr_yaw = interp1(TR.time, TR.yaw, time, 'linear', NaN);

            fh_att = figure('Name', 'Attitude (roll, pitch) [deg]');
            plot(time, T.roll, '-','Color',[0 0 1],'LineWidth',1.5); hold on; % roll: blue
            plot(time, T.pitch, '-','Color',[1 0 0],'LineWidth',1.5);        % pitch: red
            plot(time, tr_roll, '--','Color',[0 0 1],'LineWidth',1.2);
            plot(time, tr_pitch, '--','Color',[1 0 0],'LineWidth',1.2);
            hold off; grid on; xlabel('時刻 [s]'); ylabel('Angle [deg]');
            legend('roll','pitch','roll\_truth','pitch\_truth','Location','best');

            % Yaw の専用 figure（推定: 実線、真値: 点線）
            % ジンバルロック対策: unwrapで連続化
            fh_yaw = figure('Name', 'Yaw [deg] (unwrapped)');
            ax = axes('Parent', fh_yaw, 'XColor', 'k', 'YColor', 'k');
            yaw_unwrapped = unwrap(T.yaw * pi/180) * 180/pi;
            tr_yaw_unwrapped = unwrap(tr_yaw * pi/180) * 180/pi;
            plot(time, yaw_unwrapped, '-r','LineWidth',1.5); hold on; % estimation: red solid
            plot(time, tr_yaw_unwrapped, '--r','LineWidth',1.2); % truth: red dashed
            hold off; grid on; xlabel('時刻 [s]'); ylabel('Yaw [deg] (unwrapped)');
            legend('yaw','yaw\_truth','Location','best');

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
