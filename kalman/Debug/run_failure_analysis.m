% 推定失敗データの分析スクリプト

clear; close all;

script_full = which('run_failure_analysis.m');
if ~isempty(script_full)
    debug_dir = fileparts(script_full);
else
    debug_dir = pwd;
end
project_dir = fileparts(debug_dir);
addpath(fullfile(project_dir, 'Debug'));

gen_data_dir = fullfile(project_dir, 'GenerateData');
results_dir = fullfile(project_dir, 'Results');

sensor_f = fullfile(gen_data_dir, 'sensor_data_f.csv');
truth_f = fullfile(gen_data_dir, 'truth_data_f.csv');
est_f = fullfile(results_dir, 'estimation_f.csv');

if exist(sensor_f,'file') && exist(truth_f,'file') && exist(est_f,'file')
    analyze_failure(sensor_f, truth_f, est_f, 'StaticTime', 2.0, 'dt', 0.0025, 'PlotResults', true);
    analyze_failure(fullfile(gen_data_dir, 'sensor_data.csv'), fullfile(gen_data_dir, 'truth_data.csv'), fullfile(results_dir, 'estimation.csv'), 'StaticTime', 2.0, 'dt', 0.0025, 'PlotResults', true);
else
    analyze_failure(fullfile(gen_data_dir, 'sensor_data.csv'), fullfile(gen_data_dir, 'truth_data.csv'), fullfile(results_dir, 'estimation.csv'), 'StaticTime', 2.0, 'dt', 0.0025, 'PlotResults', true);
end

fprintf('\n分析完了。\n');
