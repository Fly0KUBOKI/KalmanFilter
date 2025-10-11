function [x_init, P_init] = robust_initialization(meas_sequence, params)
% Robust initialization strategy that uses early measurements to set initial state
% Inputs:
%   meas_sequence: cell array or struct array of first few measurements
%   params: filter parameters
% Outputs:
%   x_init: robust initial state estimate
%   P_init: robust initial covariance

% Default fallback values
x_init = zeros(10,1);
x_init(3) = 1; % small non-zero velocity
P_init = diag([100,100,25,25,10,10,10,10,10,10]); % conservative uncertainty

% If we have measurement sequence, use it for better initialization
if nargin >= 1 && ~isempty(meas_sequence)
    valid_pos = [];
    valid_vel = [];
    
    % Extract valid position measurements from first few steps
    if iscell(meas_sequence)
        for i = 1:min(5, length(meas_sequence)) % use first 5 measurements
            meas = meas_sequence{i};
            if isfield(meas, 'gps') && ~isempty(meas.gps) && all(isfinite(meas.gps))
                valid_pos = [valid_pos; meas.gps(:)'];
            end
            if isfield(meas, 'vel') && ~isempty(meas.vel) && all(isfinite(meas.vel))
                valid_vel = [valid_vel; meas.vel(:)'];
            end
        end
    elseif isstruct(meas_sequence)
        % Handle struct with arrays
        if isfield(meas_sequence, 'pos') && size(meas_sequence.pos, 1) >= 1
            for i = 1:min(5, size(meas_sequence.pos, 1))
                if all(isfinite(meas_sequence.pos(i,:)))
                    valid_pos = [valid_pos; meas_sequence.pos(i,:)];
                end
            end
        end
        if isfield(meas_sequence, 'vel') && size(meas_sequence.vel, 1) >= 1
            for i = 1:min(5, size(meas_sequence.vel, 1))
                if all(isfinite(meas_sequence.vel(i,:)))
                    valid_vel = [valid_vel; meas_sequence.vel(i,:)];
                end
            end
        end
    end
    
    % Use valid measurements for initialization
    if ~isempty(valid_pos)
        x_init(1:2) = mean(valid_pos, 1)'; % average position
        pos_std = std(valid_pos, 0, 1);
        if all(pos_std > 0)
            P_init(1,1) = max(pos_std(1)^2, 1); % at least 1m uncertainty
            P_init(2,2) = max(pos_std(2)^2, 1);
        end
    end
    
    if ~isempty(valid_vel)
        x_init(3:4) = mean(valid_vel, 1)'; % average velocity
        vel_std = std(valid_vel, 0, 1);
        if all(vel_std > 0)
            P_init(3,3) = max(vel_std(1)^2, 0.25); % at least 0.5m/s uncertainty
            P_init(4,4) = max(vel_std(2)^2, 0.25);
        end
    elseif size(valid_pos, 1) >= 2 && isfield(params, 'dt')
        % Estimate initial velocity from position differences
        dt = params.dt;
        vel_est = diff(valid_pos) / dt;
        if ~isempty(vel_est)
            x_init(3:4) = mean(vel_est, 1)';
            P_init(3,3) = max(var(vel_est(:,1)), 1);
            P_init(4,4) = max(var(vel_est(:,2)), 1);
        end
    end
end

% Apply parameter overrides if available
if nargin >= 2 && ~isempty(params)
    if isfield(params, 'kf') && isfield(params.kf, 'x0') && ~isempty(params.kf.x0)
        % Only override if user explicitly provided x0
        x_init = params.kf.x0(:);
    end
    if isfield(params, 'kf') && isfield(params.kf, 'P0') && ~isempty(params.kf.P0)
        % Only override if user explicitly provided P0
        P_init = params.kf.P0;
    end
end

% Final sanity checks
x_init(~isfinite(x_init)) = 0;
P_init(~isfinite(P_init)) = 1;
P_init = ensure_positive_definite(P_init);

end

function P = ensure_positive_definite(P)
% Ensure matrix is positive definite
[V, D] = eig(P);
D = diag(D);
D(D <= 0) = eps; % replace non-positive eigenvalues
P = V * diag(D) * V';
end