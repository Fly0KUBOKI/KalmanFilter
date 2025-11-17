function plot_trajectory()
% PLOT_TRAJECTORY - 推定軌跡と真値軌跡の可視化
%
% 使用法:
%   plot_trajectory()
%
% 必要ファイル:
%   - Results/estimation.csv
%   - GenerateData/truth_data.csv
%
% 出力:
%   3D軌跡のプロット (推定 vs 真値)

    %% ファイル読み込み
    try
        est_data = readtable('Results/estimation.csv');
        truth_data = readtable('GenerateData/truth_data.csv');
    catch ME
        error('ファイル読み込み失敗: %s', ME.message);
    end
    
    %% データサイズ確認
    n = min(height(est_data), height(truth_data));
    
    %% Position データ
    est_x = est_data.pos_x(1:n);
    est_y = est_data.pos_y(1:n);
    est_z = est_data.pos_z(1:n);
    
    true_x = truth_data.x(1:n);
    true_y = truth_data.y(1:n);
    true_z = truth_data.z(1:n);
    
    %% 3Dプロット作成
    figure('Name', 'Trajectory Comparison', 'NumberTitle', 'off', ...
        'Position', [100, 100, 1000, 800]);
    
    hold on;
    plot3(true_x, true_y, true_z, 'b-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    plot3(est_x, est_y, est_z, 'r--', 'LineWidth', 1.5, 'DisplayName', 'ESKF Estimate');
    
    % スタート/ゴール地点
    plot3(true_x(1), true_y(1), true_z(1), 'go', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(true_x(end), true_y(end), true_z(end), 'rs', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'r', 'DisplayName', 'End');
    
    hold off;
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    title('3D Trajectory: ESKF vs Ground Truth');
    legend('Location', 'best');
    axis equal;
    view(45, 30);
    
    %% 統計情報
    pos_error = sqrt((est_x - true_x).^2 + (est_y - true_y).^2 + (est_z - true_z).^2);
    fprintf('\n--- Trajectory Statistics ---\n');
    fprintf('Position RMS Error: %.4f m\n', rms(pos_error));
    fprintf('Max Position Error: %.4f m\n', max(pos_error));
    fprintf('Mean Position Error: %.4f m\n', mean(pos_error));
end
