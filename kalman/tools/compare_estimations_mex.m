function results = compare_estimations_mex(matlabFile, mexFile, outCsv)
%COMPARE_ESTIMATIONS_MEX Compare MATLAB vs MEX estimation CSVs
%  results = compare_estimations_mex(matlabFile, mexFile, outCsv)
%  If files are omitted, defaults to Results/estimation_matlab.csv and
%  Results/estimation_mex.csv. Writes summary to CSV if outCsv provided.

if nargin<1 || isempty(matlabFile)
    matlabFile = fullfile(pwd,'Results','estimation_matlab.csv');
end
if nargin<2 || isempty(mexFile)
    mexFile = fullfile(pwd,'Results','estimation_mex.csv');
end
if nargin<3
    outCsv = fullfile('Results','estimation_comparison_mex_vs_matlab.csv');
end

Tm = readtable(matlabFile);
Tx = readtable(mexFile);

if ~ismember('time',Tm.Properties.VariableNames) || ~ismember('time',Tx.Properties.VariableNames)
    error('Both files must contain a ''time'' column');
end

% find common times and align
[common, ia, ib] = intersect(Tm.time, Tx.time);
if isempty(common)
    error('No common time stamps between files');
end

% collect variable names (exclude time)
cols = Tm.Properties.VariableNames;
cols(strcmp(cols,'time')) = [];

valsM = table2array(Tm(ia, cols));
valsX = table2array(Tx(ib, cols));

err = valsX - valsM; % error = MEX - MATLAB

rmse = sqrt(mean(err.^2,1));
mae = mean(abs(err),1);
std_err = std(err,0,1);
overall_rmse = sqrt(mean(err(:).^2));

% print concise summary
fprintf('Comparison: %s vs %s\n', matlabFile, mexFile);
fprintf('Aligned samples: %d\n', numel(common));
fprintf('Overall RMSE (all states): %.6g\n', overall_rmse);
fprintf('\nPer-state metrics:\n');
fprintf('%-20s %-12s %-12s %-12s\n','state','rmse','mae','std_err');
for i=1:numel(cols)
    fprintf('%-20s %-12.6g %-12.6g %-12.6g\n', cols{i}, rmse(i), mae(i), std_err(i));
end

% build results struct and table
results.cols = cols(:);
results.rmse = rmse(:);
results.mae = mae(:);
results.std_err = std_err(:);
results.overall_rmse = overall_rmse;

T = table(results.cols, results.rmse, results.mae, results.std_err, ...
    'VariableNames',{'state','rmse','mae','std_err'});

try
    writetable(T, outCsv);
    fprintf('Wrote summary CSV: %s\n', outCsv);
catch
    warning('Could not write summary CSV to %s', outCsv);
end

end
