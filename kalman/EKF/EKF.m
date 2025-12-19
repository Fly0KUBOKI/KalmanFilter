classdef EKF < KF
    % EKF  標準拡張カルマンフィルタクラス（KF を継承）
    %
    % スキャフォールド実装: predict/update/getState を @EKF に分離

    properties
        x    % 状態ベクトル
    end

    methods
        function obj = EKF(x0, P0, params)
            if nargin < 1 || isempty(x0)
                obj.x = [];
            else
                obj.x = x0;
            end
            if nargin >= 2 && ~isempty(P0)
                obj.P = P0; % KF.P
            end
            if nargin >= 3 && ~isempty(params)
                obj.params = params;
            end
        end

        % メソッドは @EKF に実装
        predict(obj, u, Q);
        update(obj, z, h_func, R);
        [x, P] = getState(obj);
    end
end
