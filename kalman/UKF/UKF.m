classdef UKF < KF
    % UKF  Unscented Kalman Filter クラス（メソッドは @UKF 内に実装）
    %
    % 使用法（例）:
    %   ukf = UKF(x0, P0, params);
    %   ukf.predict(@(x) f(x,u), Q);
    %   ukf.update(z, @(x) h(x), R);

    properties
        x          % 状態ベクトル (nx1)
        n          % 状態次元

        % UKF パラメータ
        alpha = 1e-3
        beta = 2
        kappa = 0

        % 任意の追加パラメータを格納する構造体
        params = struct()
    end

    methods
        % コンストラクタ
        function obj = UKF(x0, P0, params)
            % UKF Construct an instance of this class
            %
            % 入力:
            %   x0     - 初期状態ベクトル
            %   P0     - 初期共分散行列
            %   params - オプション構造体 (alpha,beta,kappa,Q,その他)

            if nargin < 1 || isempty(x0)
                obj.x = [];
                obj.n = 0;
            else
                obj.x = x0;
                obj.n = length(x0);
            end

            if nargin < 2 || isempty(P0)
                if obj.n > 0
                    obj.P = eye(obj.n);
                else
                    obj.P = [];
                end
            else
                obj.P = P0;
            end

            if nargin >= 3 && ~isempty(params)
                if isfield(params,'alpha'), obj.alpha = params.alpha; end
                if isfield(params,'beta'), obj.beta = params.beta; end
                if isfield(params,'kappa'), obj.kappa = params.kappa; end
                if isfield(params,'Q'), obj.Q = params.Q; end
                obj.params = params;
            end
        end

        % 予測ステップ: f_handle は状態遷移関数ハンドル (x->x_next)
        predict(obj, f_handle, Q);

        % 観測更新: h_handle は観測関数ハンドル (x->z_pred)
        % 戻り値は [K, S, y] を返す実装にする
        update(obj, z, h_handle, R);

        % 状態取得
        [x, P] = getState(obj);
    end
end
