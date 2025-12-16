classdef NoOpGyroFilter < handle
    % NOOPGYROFILTER  ジャイロフィルタの no-op シム (削除後の互換用)
    %
    % 既存コードが参照する `w_filtered` プロパティと `apply` メソッドを提供します。
    properties
        w_filtered = [0;0;0];
        w_filtered_axis = [0;0;0];
        bias_estimate = [0;0;0];
        noise_history = [];
    end

    methods
        function obj = NoOpGyroFilter(config)
            % コンストラクタは引数を無視して初期化のみ行う
            if nargin < 1
                config = struct();
            end
            obj.w_filtered = [0;0;0];
            obj.w_filtered_axis = [0;0;0];
            obj.bias_estimate = [0;0;0];
            obj.noise_history = [];
        end

        function [w_out, is_outlier, info] = apply(obj, w_meas, w_expected)
            % APPLY  入力をそのまま返す（no-op）
            if nargin < 3, w_expected = zeros(3,1); end
            w_out = w_meas;
            obj.w_filtered = w_out;
            obj.w_filtered_axis = w_out;
            is_outlier = false;
            info = struct('residual_norm', norm(w_out - w_expected), 'bias_estimate', obj.bias_estimate);
            % 更新履歴
            obj.noise_history = [obj.noise_history; info.residual_norm];
            if numel(obj.noise_history) > 1000
                obj.noise_history = obj.noise_history(end-999:end);
            end
        end

        function noise_level = getNoiseLevel(obj)
            if isempty(obj.noise_history)
                noise_level = 0.0;
            else
                noise_level = std(obj.noise_history);
            end
        end

        function bias = getBiasEstimate(obj)
            bias = obj.bias_estimate;
        end
    end
end
