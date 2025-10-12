function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = eskf_filter_step_wrapper(x_prev, P_prev, meas, params)
% Wrapper to call the full ESKF implementation from 8-dim state representation
% Adds a simple pragmatic 8<->15 covariance mapping to preserve consistency
% between the lightweight 8-state interface and the ESKF 15x15 error-state.
%
% Inputs:
%  - x_prev: 8x1 state [v; q1; q2; q3; q4; x; y; z]
%  - P_prev: 8x8 covariance
%  - meas: measurement struct passed to eskf_filter_step
%  - params: configuration
%
% Outputs use the generic filter_step signature so consumers can be unified.

% prepare generic outputs
x_pred = [];
P_pred = [];
y = [];
S = [];
K = [];

if nargin < 4
    error('eskf_filter_step_wrapper requires x_prev, P_prev, meas, params');
end
if isempty(x_prev) || numel(x_prev) < 8
    error('x_prev must be 8x1 [v q1 q2 q3 q4 x y z]');
end

% Map 8-dim to nominal ESKF state
nominal = struct();
nominal.pos = x_prev(6:8);
q = x_prev(2:5);
nominal.quat = q(:);
qw = q(1); qx = q(2); qy = q(3); qz = q(4);
R = [1-2*(qy^2+qz^2), 2*(qx*qy- qz*qw), 2*(qx*qz+ qy*qw);
     2*(qx*qy+ qz*qw), 1-2*(qx^2+qz^2), 2*(qy*qz- qx*qw);
     2*(qx*qz- qy*qw), 2*(qy*qz+ qx*qw), 1-2*(qx^2+qy^2)];
v = x_prev(1);
nominal.vel = R * [v;0;0];
nominal.bg = zeros(3,1);
nominal.ba = zeros(3,1);

% --- build a pragmatic 15x15 P_eskf from 8x8 P_prev ---
% P_eskf layout: [pos(3); vel(3); theta(3); bg(3); ba(3)]
P_eskf = zeros(15);
try
    if ~isempty(P_prev) && all(size(P_prev) == [8 8])
        % position covariance from P_prev(6:8,6:8) if available
        P_eskf(1:3,1:3) = P_prev(6:8,6:8);
        % velocity covariance: project scalar speed variance along heading
        var_v = max(1e-6, P_prev(1,1));
        e = R(:,1); % vehicle forward direction in world
        P_eskf(4:6,4:6) = var_v * (e * e') + 1e-3 * eye(3);
        % attitude (small-angle) variance approximate from quaternion components
        if all(size(P_prev,2) >= 5)
            % use mean variance of quaternion vector part (q2..q4)
            qcov = P_prev(3:5,3:5);
            var_theta = max(1e-6, mean(diag(qcov)));
        else
            var_theta = 1e-3;
        end
        P_eskf(7:9,7:9) = var_theta * eye(3);
        % biases: set conservative small variances
        P_eskf(10:12,10:12) = (1e-2) * eye(3);
        P_eskf(13:15,13:15) = (1e-2) * eye(3);
    else
        % fallback conservative default
        P_eskf = diag([1,1,1, 0.1,0.1,0.1, 1e-3,1e-3,1e-3, 1e-2,1e-2,1e-2, 1e-2,1e-2,1e-2]);
    end
catch
    % ensure P_eskf is valid
    P_eskf = eye(15) * 1e-3;
end

% Call ESKF core
[nominal_out, P_eskf_out, innovations] = eskf_filter_step(nominal, P_eskf, meas, params);

% Map nominal_out back to 8-dim
x_upd = zeros(8,1);
x_upd(6:8) = nominal_out.pos(:);
if isfield(nominal_out,'quat') && numel(nominal_out.quat) >= 4
    x_upd(2:5) = nominal_out.quat(:);
else
    x_upd(2:5) = [1;0;0;0];
end
x_upd(1) = norm(nominal_out.vel(:));

% Map 15x15 P back to 8x8 P_upd
P_upd = zeros(8);
try
    % pos block
    P_upd(6:8,6:8) = P_eskf_out(1:3,1:3);
    % speed variance = projection of vel-cov onto heading
    e_out = eskf_utils('quat_to_rotm', x_upd(2:5)) * [1;0;0];
    pv = e_out(:)' * P_eskf_out(4:6,4:6) * e_out(:);
    P_upd(1,1) = max(pv, 1e-8);
    % quaternion block: approximate map from small-angle covariance
    var_theta_out = mean(diag(P_eskf_out(7:9,7:9)));
    % spread into 4-element quaternion covariance as small diagonal
    P_upd(2:5,2:5) = eye(4) * max(var_theta_out/4, 1e-8);
catch
    % fallback: preserve previous P_prev if possible
    if ~isempty(P_prev) && all(size(P_prev) == [8 8])
        P_upd = P_prev;
    else
        P_upd = diag([1,1e-3,1e-3,1e-3,1e-3,10,10,10]);
    end
end

% return params for compatibility
params = params;
end
