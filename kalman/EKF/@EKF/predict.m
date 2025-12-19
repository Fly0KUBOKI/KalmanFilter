function predict(obj, f_handle, Q)
    % EKF 予測ステップ（ユーザが与える f_handle を用いる）
    if isempty(obj.x) || isempty(obj.P)
        error('EKF:predict:EmptyState','State x and P must be initialized before predict');
    end

    % 単純に f_handle を呼んで状態を更新
    x_pred = f_handle(obj.x);
    % 線形化は外部で行う（kalman_filter_core.compute_jacobian を使ってください）
    if nargin >= 3 && ~isempty(Q)
        obj.P = obj.P + Q;
    end
    obj.x = x_pred;
end
