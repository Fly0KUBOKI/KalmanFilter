function compare_numeric_results(this_pc_file, other_pc_file)
% Phase 1: This PC と Other PC の数値結果を比較
%
% Usage:
%   compare_numeric_results('Results/phase1_numeric_test.mat', ...
%                          'OtherPC/Results/phase1_numeric_test.mat')

fprintf('====================================\n');
fprintf('CROSS-PC NUMERIC COMPARISON\n');
fprintf('====================================\n\n');

% ファイル読み込み
this_pc = load(this_pc_file);
other_pc = load(other_pc_file);

if length(this_pc.results) ~= length(other_pc.results)
    error('Results have different number of seeds');
end

fprintf('%-10s %-20s %-20s %-15s\n', 'Seed', 'This PC Pos RMSE', 'Other PC Pos RMSE', 'Difference (m)');
fprintf(repmat('-', 1, 80)); fprintf('\n');

max_diff = 0;
for i = 1:length(this_pc.results)
    seed = this_pc.results(i).seed;
    pos_this = this_pc.results(i).pos_rmse;
    pos_other = other_pc.results(i).pos_rmse;
    diff = abs(pos_this - pos_other);
    
    fprintf('%-10d %.15f  %.15f  %.2e\n', seed, pos_this, pos_other, diff);
    
    if diff > max_diff
        max_diff = diff;
    end
end

fprintf(repmat('-', 1, 80)); fprintf('\n');
fprintf('Maximum difference: %.2e m\n', max_diff);

% 判定
threshold = 1e-10;
if max_diff < threshold
    fprintf('✅ PASS: Numeric consistency achieved (< %.2e m)\n', threshold);
else
    fprintf('❌ FAIL: Numeric difference too large (> %.2e m)\n', threshold);
    fprintf('    Possible causes:\n');
    fprintf('    - float/double conversion not fully eliminated\n');
    fprintf('    - Compiler optimization differences\n');
    fprintf('    - Uninitialized variable usage\n');
end

end
