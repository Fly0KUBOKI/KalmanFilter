function [K, S, y] = update(obj, z, h_handle, R)
    % UPDATE  UKF 観測更新ラッパー（ukf_update を利用）
    %
    % 入力:
    %   z        - 観測ベクトル
    %   h_handle - 観測関数ハンドル: z_pred = h_handle(x)
    %   R        - 観測ノイズ共分散
    %
    % 出力:
    %   K, S, y  - カルマンゲイン, イノベーション共分散, イノベーション

    if isempty(obj.x) || isempty(obj.P)
        error('UKF:update:EmptyState','State x and P must be initialized before update');
    end

    % ukf_update expects x as column vector and P
    [x_upd, P_upd, K, S, y] = ukf_update(obj.x, obj.P, z, h_handle, R, obj.alpha, obj.beta, obj.kappa);

    obj.x = x_upd;
    obj.P = (P_upd + P_upd')/2;
end
