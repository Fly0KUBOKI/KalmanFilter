function predict(obj, f_handle, Q)
    % PREDICT  UKF 予測ステップ
    %
    % 入力:
    %   f_handle - 状態遷移関数ハンドル: x_next = f_handle(x)
    %   Q        - プロセスノイズ共分散 (nxn)

    if isempty(obj.x) || isempty(obj.P)
        error('UKF:predict:EmptyState','State x and P must be initialized before predict');
    end

    if nargin < 3 || isempty(Q)
        Q = obj.Q;
    end

    % シグマポイント生成
    [sig, wm, wc] = ukf_sigma_points(obj.x, obj.P, obj.alpha, obj.beta, obj.kappa);
    n_sig = size(sig,2);

    % シグマポイント伝播
    x_pred_sigma = zeros(size(sig));
    for i = 1:n_sig
        x_pred_sigma(:,i) = f_handle(sig(:,i));
    end

    % 平均の計算
    x_pred = x_pred_sigma * wm;

    % 共分散の計算
    P_pred = zeros(size(obj.P));
    for i = 1:n_sig
        dx = x_pred_sigma(:,i) - x_pred;
        P_pred = P_pred + wc(i) * (dx * dx');
    end

    if ~isempty(Q)
        P_pred = P_pred + Q;
    end

    obj.x = x_pred;
    obj.P = (P_pred + P_pred')/2; % 対称化
end
