function varargout = plot_csv(filePath, mode)
    % PLOT_CSV 最小限の可視化（モード切替、エラーハンドリング無し）
    % modes: 'time' (default), 'pos_xy', 'vel_xy', 'att_xy'

    if nargin<1 || isempty(filePath)
        filePath = fullfile(fileparts(mfilename('fullpath')),'..','Results','estimation.csv');
    end
    if nargin<2 || isempty(mode)
        mode = 'time';
    end

    T = readtable(filePath);

    switch mode
        case 'time'
            time = T.time;
            fh = figure;
            subplot(3,1,1);
            plot(time, T.px, time, T.py, time, T.pz);
            legend('px','py','pz'); title('Position [m]'); grid on;

            subplot(3,1,2);
            plot(time, T.vx, time, T.vy, time, T.vz);
            legend('vx','vy','vz'); title('Velocity [m/s]'); grid on;

            subplot(3,1,3);
            plot(time, T.roll, time, T.pitch, time, T.yaw);
            legend('roll','pitch','yaw'); title('Attitude [rad]'); grid on;

        case 'pos'
            fh = figure;
            X = T.px; Y = T.py;
            % 固定された軸範囲（表示中に動かさない）
            xmin = min(X); xmax = max(X); ymin = min(Y); ymax = max(Y);
            dx = max(xmax-xmin, 1e-3); dy = max(ymax-ymin, 1e-3);
            pad = 0.05 * max(dx, dy);
            axis([xmin-pad xmax+pad ymin-pad ymax+pad]);
            hold on; grid on; axis equal;
            hpath = plot(NaN, NaN, '-b');
            hcur = plot(NaN, NaN, 'ro');
            plot(X(1), Y(1), 'go'); % start
            plot(X(end), Y(end), 'ko'); % end (keep end marker)
            xlabel('x [m]'); ylabel('y [m]');
            for i = 1:numel(X)
                set(hpath, 'XData', X(1:i), 'YData', Y(1:i));
                set(hcur, 'XData', X(i), 'YData', Y(i));
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
    end

    if nargout>0
        varargout{1} = fh;
    end
end
