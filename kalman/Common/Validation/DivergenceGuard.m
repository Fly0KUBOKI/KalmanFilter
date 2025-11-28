classdef DivergenceGuard < handle
    % DivergenceGuard 発散防止機能
    
    properties
        config
    end
    
    methods
        function obj = DivergenceGuard(config)
            obj.config = config;
        end
        
        function P_reg = regularize_covariance(~, P, eps_val)
            if nargin < 3
                eps_val = 1e-9;
            end
            % 対称化
            P_reg = (P + P') / 2;
            % 対角項に小さな値を追加
            P_reg = P_reg + eye(size(P)) * eps_val;
        end
        
        function P_reg = regularize_for_ukf(obj, P)
            P_reg = obj.regularize_covariance(P, 1e-9);
        end
        
        function K_clamped = clamp_gain(obj, K)
            % ゲインを制限
            max_gain = obj.config.max_gain_norm;
            K_clamped = K;
            for i = 1:size(K,1)
                row_norm = norm(K(i,:));
                if row_norm > max_gain
                    K_clamped(i,:) = K(i,:) * (max_gain / row_norm);
                end
            end
        end
        
        function [v_clip, P_clip, clipped] = check_and_clip_velocity(obj, v, P, idx)
            max_v = obj.config.max_velocity;
            v_clip = v;
            P_clip = P;
            clipped = false;
            
            v_norm = norm(v);
            if v_norm > max_v
                v_clip = v * (max_v / v_norm);
                clipped = true;
            end
        end
    end
end
