% test_set_mag_ref.m
params = config_params();
params.initial.mag_ref = [0, 50, 0];
fname = generate_and_save_data(params, fullfile(pwd, 'sim_data_magtest.csv'));
T = readtable(fname);
fprintf('Saved %s, first mag row: mag3_x=%.3f, mag3_y=%.3f, mag3_z=%.3f\n', fname, T.mag3_x(1), T.mag3_y(1), T.mag3_z(1));
