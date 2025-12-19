function [K,S,y] = update(obj, z, h_func, R)
    % EKF 更新（簡易ラッパー）
    if isempty(obj.x) || isempty(obj.P)
        error('EKF:update:EmptyState','State x and P must be initialized before update');
    end

    % 予測観測
    z_pred = h_func(obj.x);
    y = z - z_pred;

    % ヤコビアンはユーザが h_func_jacobian として渡す設計でも良いが、
    % ここでは kalman_filter_core の compute_innovation_and_S を使うための
    % 最小限のラッパーに留める。
    H = []; S = [];
    K = [];
    % 実装の詳細はプロジェクト要件に合わせて拡張してください.
end
