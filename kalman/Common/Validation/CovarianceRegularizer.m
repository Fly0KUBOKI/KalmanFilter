classdef CovarianceRegularizer
    % COVARIANCEREGULARIZER 共分散正則化の静的メソッド集
    % C++移行を想定した設計
    
    methods (Static)
        %% 共分散行列の正則化
        function P = regularize(P, min_var, max_var)
            % 共分散行列の正則化
            % 入力:
            %   P: 共分散行列 [nxn]
            %   min_var: 最小分散（対角要素の下限）
            %   max_var: 最大分散（対角要素の上限）
            % 出力:
            %   P: 正則化された共分散行列
            
            if nargin < 2 || isempty(min_var)
                min_var = 1e-9;
            end
            if nargin < 3 || isempty(max_var)
                max_var = inf;
            end
            
            % 対称性の強制
            P = CovarianceRegularizer.enforce_symmetry(P);
            
            % 対角要素の制限
            for i = 1:size(P, 1)
                if P(i,i) < min_var
                    P(i,i) = min_var;
                elseif P(i,i) > max_var
                    P(i,i) = max_var;
                end
            end
            
            % 正定値性の確保
            P = CovarianceRegularizer.ensure_positive_definite(P);
        end
        
        function P = enforce_symmetry(P)
            % 共分散行列の対称性を強制
            P = 0.5 * (P + P');
        end
        
        function P = ensure_positive_definite(P, epsilon)
            % 共分散行列の正定値性を確保
            % 固有値が負または小さすぎる場合に正則化
            if nargin < 2
                epsilon = 1e-9;
            end
            
            % 固有値分解
            [V, D] = eig(P);
            d = diag(D);
            
            % 負の固有値を正則化
            min_eig = min(d);
            if min_eig < epsilon
                d = max(d, epsilon);
                P = V * diag(d) * V';
                P = CovarianceRegularizer.enforce_symmetry(P);
            end
        end
        
        function P = add_process_noise(P, Q)
            % プロセスノイズを追加（数値安定性向上）
            P = P + Q;
            P = CovarianceRegularizer.enforce_symmetry(P);
        end
        
        %% カルマンゲインの制限
        function K = clamp_gain(K, max_gain)
            % カルマンゲインの要素ごとの制限
            % 入力:
            %   K: カルマンゲイン [nxm]
            %   max_gain: 最大ゲイン値
            % 出力:
            %   K: 制限されたゲイン
            
            if nargin < 2 || isempty(max_gain)
                max_gain = 100;
            end
            
            K = max(min(K, max_gain), -max_gain);
        end
        
        function K = clamp_gain_norm(K, max_norm)
            % カルマンゲインのFrobeniusノルム制限
            % 入力:
            %   K: カルマンゲイン [nxm]
            %   max_norm: 最大ノルム
            % 出力:
            %   K: 制限されたゲイン
            
            K_norm = norm(K, 'fro');
            if K_norm > max_norm
                K = K * (max_norm / K_norm);
            end
        end
        
        function K = clamp_gain_rows(K, max_gain, row_indices)
            % 特定行のゲイン制限（例: 姿勢部分のみ）
            % 入力:
            %   K: カルマンゲイン [nxm]
            %   max_gain: 最大ゲイン値
            %   row_indices: 制限する行のインデックス
            % 出力:
            %   K: 制限されたゲイン
            
            K(row_indices, :) = max(min(K(row_indices, :), max_gain), -max_gain);
        end
        
        %% Joseph form 更新（数値安定性向上）
        function P = joseph_form_update(P, K, H, R)
            % Joseph形式による共分散更新（数値安定性が高い）
            % P+ = (I - K*H) * P * (I - K*H)' + K*R*K'
            n = size(P, 1);
            I = eye(n);
            IKH = I - K * H;
            
            P = IKH * P * IKH' + K * R * K';
            P = CovarianceRegularizer.enforce_symmetry(P);
        end
        
        %% イノベーション共分散の正則化
        function S = regularize_innovation_covariance(S, min_var)
            % イノベーション共分散の正則化
            if nargin < 2
                min_var = 1e-12;
            end
            
            S = CovarianceRegularizer.enforce_symmetry(S);
            
            % 対角要素の下限
            for i = 1:size(S, 1)
                if S(i,i) < min_var
                    S(i,i) = min_var;
                end
            end
        end
        
        %% UKF用の正則化
        function P = regularize_for_ukf(P, min_var)
            % UKFのシグマポイント生成用に正則化
            % Cholesky分解が確実に成功するように調整
            if nargin < 2
                min_var = 1e-6;
            end
            
            P = CovarianceRegularizer.enforce_symmetry(P);
            P = CovarianceRegularizer.ensure_positive_definite(P, min_var);
            
            % Cholesky分解のテスト
            try
                chol(P);
            catch
                % 失敗した場合は追加の正則化
                P = P + eye(size(P)) * min_var;
            end
        end
        
        %% 部分行列の正則化
        function P = regularize_submatrix(P, indices, min_var, max_var)
            % 共分散行列の特定部分を正則化
            % 例: 姿勢部分のみ制限
            
            if nargin < 3
                min_var = 1e-9;
            end
            if nargin < 4
                max_var = inf;
            end
            
            for i = indices
                if P(i,i) < min_var
                    P(i,i) = min_var;
                elseif P(i,i) > max_var
                    P(i,i) = max_var;
                end
            end
        end
        
        %% 条件数チェック
        function [is_well_conditioned, cond_number] = check_condition_number(P, max_cond)
            % 共分散行列の条件数チェック
            if nargin < 2
                max_cond = 1e10;
            end
            
            cond_number = cond(P);
            is_well_conditioned = cond_number < max_cond;
        end
        
        function P = improve_condition_number(P, target_cond)
            % 条件数を改善（Tikhonov正則化）
            if nargin < 2
                target_cond = 1e6;
            end
            
            current_cond = cond(P);
            if current_cond > target_cond
                % 正則化パラメータを計算
                lambda = trace(P) / size(P, 1) * 1e-6;
                P = P + lambda * eye(size(P));
            end
        end
    end
end
