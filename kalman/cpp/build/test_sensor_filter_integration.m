function test_sensor_filter_integration()
% Compare mex_sensor_filter (if present) with mex_run_eskf('sensor_filter_update')
sensor.accel = [0;0;9.8];
sensor.gyro = [0;0;0];
sensor.mag = [30;0;0];
sensor.gps_pos = [0;0;0];
sensor.alt_baro = 1013.25;
sensor.dt = 0.01;

% Reference (if mex_sensor_filter exists)
has_ref = (exist('mex_sensor_filter','file') == 3);
if has_ref
    try
        ref = mex_sensor_filter('update', sensor);
    catch ME
        disp('mex_sensor_filter update failed:'); disp(ME.message); has_ref = false;
    end
end

try
    out = mex_run_eskf('sensor_filter_update', sensor);
catch ME
    disp('mex_run_eskf sensor_filter_update failed:'); disp(ME.message); return;
end

if has_ref
    % compare accel and mag
    da = norm(ref.accel - out.accel);
    dm = norm(ref.mag - out.mag);
    fprintf('accel diff: %g, mag diff: %g\n', da, dm);
    if da < 1e-6 && dm < 1e-6
        disp('✓ Sensor filter integration OK');
    else
        disp('✗ Sensor filter integration mismatch');
    end
else
    disp('Reference mex_sensor_filter not available; output shown:');
    disp(out);
end
end
