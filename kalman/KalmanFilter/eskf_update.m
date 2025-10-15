function [p, v, q, ba, bg, P, status] = eskf_update(p, v, q, ba, bg, P, obs, settings, k)
% ESKF_UPDATE  Perform one ESKF step for time index k
% Inputs:
%  p,v,q,ba,bg,P - current nominal state and covariance
%  obs - observation struct (as returned by read_csv)
%  settings - settings struct (dt, Q, R_*, freq_*)
%  k - time index (integer)
% Outputs:
%  updated p,v,q,ba,bg,P and status struct (optional diagnostics)

if nargin < 9
    error('eskf_update: requires p,v,q,ba,bg,P,obs,settings,k');
end

dt = settings.dt;
g = [0;0;9.81];

% extract measurements at time k
a = [obs.ax(k); obs.ay(k); obs.az(k)];
w = [obs.wx(k); obs.wy(k); obs.wz(k)];
% sensor CSV provides gyro in deg/s; convert to rad/s for internal use
w = deg2rad(w);

% nominal integration
[p, v, q, ba, bg] = integrate_nominal(p, v, q, ba, bg, a, w, dt, g);

% predict covariance
P = predict_step(P, q, a, ba, w, bg, settings.Q, dt);

% accel update (roll/pitch when stationary)
[p, v, q, ba, bg, P] = update_accel(p, v, q, ba, bg, P, a, dt);

% periodic updates
if isfield(settings,'freq_mag') && mod(k, settings.freq_mag) == 0
    [q, P] = update_mag(q, P, [obs.mx(k); obs.my(k); obs.mz(k)]);
end
if isfield(settings,'freq_baro') && mod(k, settings.freq_baro) == 0
    [p, P] = update_baro(p, P, obs.pressure(k));
end
if isfield(settings,'freq_gps') && mod(k, settings.freq_gps) == 0 && ~isnan(obs.lat(k)) && ~isnan(obs.lon(k))
    if ~isfield(settings,'gps_origin')
        settings.gps_origin = [obs.lat(1); obs.lon(1); obs.alt(1)];
    end
    
    gps_dt = settings.dt * settings.freq_gps;
    [p, v, q, ba, bg, P] = update_gps(p, v, q, ba, bg, P, obs.lat(k), obs.lon(k), obs.alt(k), settings.gps_origin, gps_dt);
end

status = struct('updated',true,'k',k);
end
