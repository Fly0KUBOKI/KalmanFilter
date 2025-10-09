function [F, Q] = build_process_model(n, dt, params)
% BUILD_PROCESS_MODEL - Construct state transition matrix F and process noise Q
%
% Inputs:
%   n: state dimension
%   dt: time step
%   params: parameter struct with params.kf.process_noise_accel
%
% Outputs:
%   F: state transition matrix (n×n)
%   Q: process noise covariance (n×n)
%
% State vector (n=10): [x, y, vx, vy, theta, ax, ay, omega, z, vz]

% Validate dt
if dt <= 0
    dt = 1e-6;  % minimum dt to avoid singularities
end

F = eye(n);
Q = eye(n) * 1e-6;  % Initialize with small non-zero values to avoid singularity

if n >= 10
    % Kinematic model
    F(1,3) = dt; F(1,6) = 0.5*dt^2;  % x = x + vx*dt + 0.5*ax*dt^2
    F(2,4) = dt; F(2,7) = 0.5*dt^2;  % y = y + vy*dt + 0.5*ay*dt^2
    F(3,6) = dt;                      % vx = vx + ax*dt
    F(4,7) = dt;                      % vy = vy + ay*dt
    F(5,8) = dt;                      % theta = theta + omega*dt
    F(9,10) = dt;                     % z = z + vz*dt
    
    % Process noise
    if isfield(params, 'kf') && isfield(params.kf, 'process_noise_accel')
        q_a = params.kf.process_noise_accel;
    else
        q_a = 0.5;  % default
    end
    
    % Position noise (small)
    Q(1,1) = 0.01; Q(2,2) = 0.01;
    
    % Velocity noise (small)
    Q(3,3) = 0.01; Q(4,4) = 0.01;
    
    % Heading noise
    Q(5,5) = deg2rad(1)^2;
    
    % Acceleration noise (main process noise)
    Q(6,6) = (q_a)^2; 
    Q(7,7) = (q_a)^2;
    
    % Angular velocity noise
    Q(8,8) = (q_a*0.1)^2;
    
    % Vertical velocity noise
    Q(10,10) = (q_a)^2;
    
    % Apply theta-velocity coupling in F (weak coupling for direction consistency)
    % This is a linearization of: vx ≈ v*cos(theta), vy ≈ v*sin(theta)
    % We add small correction terms based on current state if available
    % For generic use, keep F linear; coupling can be added externally if needed
end

end
