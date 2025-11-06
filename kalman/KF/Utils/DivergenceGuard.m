classdef DivergenceGuard < handle
    %DivergenceGuard カルマンフィルタの発散を防止するユーティリティクラス
    %   各種センサー更新時のイノベーションチェック、共分散行列の正則化、
    %   状態変数のクリッピングなどの発散対策機能を提供
    
    properties (Access = private)
        % 更新履歴
        prev_innovations  % 各センサーの前回イノベーション
        update_counts     % 各センサーの更新回数
        
        % 閾値設定
        max_allowed_innov  % イノベーション上限 [m or rad]
        innov_change_ratio_threshold  % イノベーション変化率閾値
        attenuation_factor  % 減衰係数
        
        % 正則化パラメータ
        min_eigenvalue_factor  % 最小固有値係数
        max_variance_factor    % 最大分散係数
        min_rcond              % 最小条件数
        jitter_base            % ジッター基本係数
        
        % 状態制限
        max_velocity      % 最大速度 [m/s]
        max_acceleration  % 最大加速度 [m/s^2]
        max_position_change  % 最大位置変化 [m]
        max_velocity_change  % 最大速度変化 [m/s]
        max_attitude_change  % 最大姿勢変化 [rad]
        % イノベーションキャップ比率（limit に対する割合。例: 0.5 -> limit/2 に縮小）
        max_innov_cap_fraction
        % divergence dump flag
        dump_saved
        % Kalman gain clamp (Frobenius norm limit). empty or Inf = no clamp
        max_gain_norm
    end
    
    methods
        function obj = DivergenceGuard(config)
            %DivergenceGuard コンストラクタ
            %   config: 設定構造体 (オプション)
            
            % デフォルト設定
            if nargin < 1 || isempty(config)
                config = struct();
            end
            
            % イノベーション関連
            obj.max_allowed_innov = get_field(config, 'max_allowed_innov', 50.0);
            obj.innov_change_ratio_threshold = get_field(config, 'innov_change_ratio_threshold', 2.0);
            obj.attenuation_factor = get_field(config, 'attenuation_factor', 0.5);
            
            % 正則化関連
            obj.min_eigenvalue_factor = get_field(config, 'min_eigenvalue_factor', 1e-8);
            obj.max_variance_factor = get_field(config, 'max_variance_factor', 1e6);
            obj.min_rcond = get_field(config, 'min_rcond', 1e-12);
            obj.jitter_base = get_field(config, 'jitter_base', 1e-6);
            
            % 状態制限関連
            obj.max_velocity = get_field(config, 'max_velocity', 2.0);
            obj.max_acceleration = get_field(config, 'max_acceleration', 2.0);
            obj.max_position_change = get_field(config, 'max_position_change', 10.0);
            obj.max_velocity_change = get_field(config, 'max_velocity_change', 5.0);
            obj.max_attitude_change = get_field(config, 'max_attitude_change', 0.5);
            obj.max_innov_cap_fraction = get_field(config, 'max_innov_cap_fraction', 1.0);
            obj.dump_saved = false;
            obj.max_gain_norm = get_field(config, 'max_gain_norm', Inf);
            
            % 履歴初期化
            obj.prev_innovations = struct();
            obj.update_counts = struct();
        end
        
    function [dx_out, should_skip, was_attenuated] = check_and_attenuate_update(obj, sensor_name, innovation, dx_in, ctx)
            %check_and_attenuate_update 更新のチェックと減衰
            %   入力:
            %     sensor_name: センサー名 ('gps', 'mag', 'baro', 'accel')
            %     innovation: 現在のイノベーション
            %     dx_in: 状態更新量
            %   出力:
            %     dx_out: (減衰後の)状態更新量
            %     should_skip: 更新をスキップすべきか
            %     was_attenuated: 減衰が適用されたか
            
            should_skip = false;
            was_attenuated = false;
            dx_out = dx_in;
            
            innov_norm = norm(innovation);
            
            % 更新カウント初期化
            if ~isfield(obj.update_counts, sensor_name)
                obj.update_counts.(sensor_name) = 0;
            end
            
            % イノベーション上限チェック --- 値に応じてキャップ（limitに張り付く）またはスキップ
            if innov_norm > obj.max_allowed_innov
                original_innov = innov_norm; %#ok<NASGU> used for possible future logging/debug
                % 大きさによる扱い: 極端に巨大な値はデータ破損とみなしてスキップ
                skip_factor = 1e6; % limit のこの倍を超えたらスキップ
                if innov_norm > obj.max_allowed_innov * skip_factor
                    fprintf('%s update skipped (corrupt/outlier): innovation=%.2e (limit: %.1f)\n', ...
                        upper(sensor_name), innov_norm, obj.max_allowed_innov);
                    % save dump if context provided and not yet saved
                    if exist('ctx','var') && isstruct(ctx)
                        try
                            obj.save_divergence_dump(sensor_name, ctx);
                        catch
                        end
                    end
                    should_skip = true;
                    return;
                end

                % ほどほどに大きい場合はイノベーションを制限して更新を続行する
                % （limit に張り付く／fraction に縮小する挙動）
                target_norm = obj.max_allowed_innov * obj.max_innov_cap_fraction;
                if target_norm <= 0
                    target_norm = obj.max_allowed_innov;
                end
                innovation = innovation * (target_norm / innov_norm);
                innov_norm = target_norm;
                was_attenuated = true;
                % 発散・エラーではあるが、更新自体は極端なジャンプを防ぐために縮小して適用
                % キャップは頻繁に発生する可能性があるため、詳細ログは抑制する
                % save dump on first attenuation if context provided
                if ~obj.dump_saved && exist('ctx','var') && isstruct(ctx)
                    try
                        obj.save_divergence_dump(sensor_name, ctx);
                        obj.dump_saved = true;
                    catch
                    end
                end
            end
            
            % 前回との変化率チェック
            if obj.update_counts.(sensor_name) > 0 && isfield(obj.prev_innovations, sensor_name)
                prev_innov = obj.prev_innovations.(sensor_name);
                innov_change = norm(innovation - prev_innov);
                prev_innov_norm = norm(prev_innov);
                
                if prev_innov_norm > 1e-6
                    change_ratio = innov_change / prev_innov_norm;
                    
                    if change_ratio > obj.innov_change_ratio_threshold
                        dx_out = dx_in * obj.attenuation_factor;
                        was_attenuated = true;
                    end
                end
            end
            
            % 履歴更新
            obj.prev_innovations.(sensor_name) = innovation;
            obj.update_counts.(sensor_name) = obj.update_counts.(sensor_name) + 1;
        end

        function K_out = clamp_gain(obj, K_in)
            % clamp_gain Kalman gain のノルムを max_gain_norm に制限
            K_out = K_in;
            if isempty(obj.max_gain_norm) || ~isfinite(obj.max_gain_norm)
                return;
            end
            fn = norm(K_in, 'fro');
            if fn > obj.max_gain_norm && fn > 0
                K_out = K_in * (obj.max_gain_norm / fn);
            end
        end

        function save_divergence_dump(obj, sensor_name, ctx)
            % save_divergence_dump ダンプを Results に保存
            try
                outdir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Results');
            catch
                outdir = fullfile(pwd, 'Results');
            end
            if ~exist(outdir, 'dir')
                mkdir(outdir);
            end
            dump.sensor = sensor_name;
            % Prefer full matrices when present; otherwise reconstruct from diagonals
            if isfield(ctx, 'k'), dump.k = ctx.k; end
            if isfield(ctx, 'z'), dump.z = ctx.z; end
            if isfield(ctx, 'h'), dump.h = ctx.h; end
            if isfield(ctx, 'y'), dump.y = ctx.y; end

            % P: full or diag
            if isfield(ctx, 'P') && ~isempty(ctx.P)
                dump.P = ctx.P;
                dump.P_diag = diag(ctx.P);
            elseif isfield(ctx, 'P_diag') && ~isempty(ctx.P_diag)
                try
                    dump.P_diag = ctx.P_diag;
                    dump.P = diag(ctx.P_diag);
                catch
                    dump.P_diag = ctx.P_diag;
                end
            end

            % R: full or diag
            if isfield(ctx, 'R') && ~isempty(ctx.R)
                dump.R = ctx.R;
                try
                    dump.R_diag = diag(ctx.R);
                catch
                end
            elseif isfield(ctx, 'R_diag') && ~isempty(ctx.R_diag)
                try
                    dump.R_diag = ctx.R_diag;
                    dump.R = diag(ctx.R_diag);
                catch
                    dump.R_diag = ctx.R_diag;
                end
            end

            % If H provided in ctx (or can be constructed from sensor), compute S and approximate K
            S = [];
            K_approx = [];
            try
                % Accept explicit H
                if isfield(ctx, 'H') && ~isempty(ctx.H) && isfield(dump, 'P') && isfield(dump, 'R')
                    S = ctx.H * dump.P * ctx.H' + dump.R;
                    Huse = ctx.H;
                else
                    % sensor-specific fallback
                    sensor_l = lower(sensor_name);
                    Huse = [];
                    if isfield(dump, 'P') && isfield(dump, 'R')
                        if contains(sensor_l, 'gps')
                            Huse = [eye(3), zeros(3,12)];
                            S = Huse * dump.P * Huse' + dump.R;
                        elseif contains(sensor_l, 'mag') && isfield(dump, 'h')
                            hh = dump.h(:);
                            skew_h = [0, -hh(3), hh(2); hh(3), 0, -hh(1); -hh(2), hh(1), 0];
                            Huse = [zeros(3,6), skew_h, zeros(3,6)];
                            S = Huse * dump.P * Huse' + dump.R;
                        elseif contains(sensor_l, 'baro')
                            Huse = [0,0,1, zeros(1,12)];
                            S = Huse * dump.P * Huse' + dump.R;
                        end
                    end
                end
            catch
                S = [];
                Huse = [];
            end

            if ~isempty(S)
                dump.S = S;
                try
                    dump.S_rcond = rcond(S);
                catch
                    dump.S_rcond = NaN;
                end
                % K approx if possible
                try
                    if ~isempty(Huse)
                        K_approx = dump.P * Huse' / S;
                        dump.K_approx = K_approx;
                    end
                catch
                end
            end

            % Include any diagnostics passed via ctx (e.g., OutlierGuard diagnostics)
            try
                if exist('ctx','var') && isstruct(ctx) && isfield(ctx, 'diagnostics')
                    dump.diagnostics = ctx.diagnostics;
                end
            catch
            end

            % Attempt to capture a small raw sensor window around ctx.k (if available)
            try
                if isfield(ctx, 'k') && ~isempty(ctx.k) && isscalar(ctx.k)
                    kidx = double(ctx.k);
                    classPath = mfilename('fullpath');
                    base = fileparts(fileparts(classPath)); % .../kalman/KF -> base = .../kalman
                    sensorFile = fullfile(base, 'GenerateData', 'sensor_data.csv');
                    if exist(sensorFile, 'file')
                        try
                            T = readtable(sensorFile);
                            Nw = 5; i0 = max(1, kidx - Nw); i1 = min(height(T), kidx + Nw);
                            dump.raw_window = T(i0:i1, :);
                        catch
                            % fallback: do nothing
                        end
                    end
                end
            catch
            end

            dump.time = datestr(now,'yyyymmdd_HHMMSS');
            fname = fullfile(outdir, sprintf('divergence_dump_%s_%s.mat', sensor_name, datestr(now,'yyyymmdd_HHMMSS')));
            try
                save(fname, 'dump');
                fprintf('Divergence dump saved to %s\n', fname);
            catch
            end
        end
        
        function P_out = regularize_covariance(obj, P_in)
            %regularize_covariance 共分散行列の正則化
            %   入力: P_in - 共分散行列
            %   出力: P_out - 正則化された共分散行列
            
            P_out = P_in;
            n = size(P_out, 1);
            
            % 1. 対称性の強制
            P_out = (P_out + P_out') / 2;
            
            % 2. Cholesky分解チェックとジッター追加
            [~, p] = chol(P_out);
            if p > 0
                base_diag = max(abs(diag(P_out)));
                if base_diag < 1e-10
                    base_diag = 1e-10;
                end
                jitter = obj.jitter_base * base_diag * eye(n);
                P_out = P_out + jitter;
            end
            
            % 3. 固有値補正
            [V, D] = eig(P_out);
            eigvals = diag(D);
            base_eig = max(abs(eigvals));
            if base_eig < 1e-10
                base_eig = 1e-10;
            end
            min_eig = obj.min_eigenvalue_factor * base_eig;
            eigvals = max(eigvals, min_eig);
            P_out = V * diag(eigvals) * V';
            
            % 4. 条件数チェック
            if rcond(P_out) < obj.min_rcond
                boost = 1e-8 * max(abs(diag(P_out)));
                P_out = P_out + boost * eye(n);
            end
            
            % 5. 対角成分の上限
            diag_P = diag(P_out);
            base_var = max(abs(diag_P));
            if base_var < 1e-10
                base_var = 1e-10;
            end
            max_var = obj.max_variance_factor * base_var;
            diag_P = min(diag_P, max_var);
            P_out(1:n+1:end) = diag_P;

            % --- Absolute per-state caps (safety net) ---
            if n == 15
                % indices: pos 1:3, vel 4:6, att 7:9, ba 10:12, bg 13:15
                pos_cap = get_field(obj, 'pos_var_cap', 1e6);
                vel_cap = get_field(obj, 'vel_var_cap', 1e4);
                att_cap = get_field(obj, 'att_var_cap', 100);
                ba_cap  = get_field(obj, 'ba_var_cap', 1e2);
                bg_cap  = get_field(obj, 'bg_var_cap', 1e-1);
                d = diag(P_out);
                d(1:3) = min(d(1:3), pos_cap);
                d(4:6) = min(d(4:6), vel_cap);
                d(7:9) = min(d(7:9), att_cap);
                d(10:12) = min(d(10:12), ba_cap);
                d(13:15) = min(d(13:15), bg_cap);
                P_out(1:n+1:end) = d;
            end
            
            % 6. 微小な非対角成分をゼロに
            mask = abs(P_out) < 1e-15;
            P_out(mask) = 0;
            
            % 7. 最終的な対称性
            P_out = (P_out + P_out') / 2;
        end
        
        function P_out = regularize_for_ukf(obj, P_in)
            %regularize_for_ukf UKF用の強力な正則化
            %   UKFのシグマポイント生成前に使用
            
            P_out = P_in;
            n = size(P_out, 1);
            
            % 対称性
            P_out = (P_out + P_out') / 2;
            
            % Cholesky分解テスト
            [~, p] = chol(P_out);
            if p > 0
                base_diag = max(abs(diag(P_out)));
                if base_diag < 1e-10
                    base_diag = 1e-10;
                end
                jitter = obj.jitter_base * base_diag * eye(n);
                P_out = P_out + jitter;
            end
            
            % 固有値補正 (UKF用はより厳しい閾値)
            [V, D] = eig(P_out);
            eigvals = diag(D);
            base_eig = max(abs(eigvals));
            if base_eig < 1e-10
                base_eig = 1e-10;
            end
            min_eig = obj.min_eigenvalue_factor * base_eig;
            eigvals = max(eigvals, min_eig);
            P_out = V * diag(eigvals) * V';
            
            % 最終対称性
            P_out = (P_out + P_out') / 2;
        end
        
        function dx_out = clip_state_change(obj, dx_in)
            %clip_state_change 状態変化量のクリッピング
            %   入力: dx_in - 状態変化量 [pos(3), vel(3), att(3), ba(3), bg(3)]
            %   出力: dx_out - クリッピング後の状態変化量
            
            dx_out = dx_in;
            
            if length(dx_in) >= 15
                % 位置変化制限
                pos_change = dx_in(1:3);
                pos_norm = norm(pos_change);
                if pos_norm > obj.max_position_change
                    dx_out(1:3) = pos_change * (obj.max_position_change / pos_norm);
                end
                
                % 速度変化制限
                vel_change = dx_in(4:6);
                vel_norm = norm(vel_change);
                if vel_norm > obj.max_velocity_change
                    dx_out(4:6) = vel_change * (obj.max_velocity_change / vel_norm);
                end
                
                % 姿勢変化制限
                att_change = dx_in(7:9);
                att_norm = norm(att_change);
                if att_norm > obj.max_attitude_change
                    dx_out(7:9) = att_change * (obj.max_attitude_change / att_norm);
                end
                
                % バイアス変化制限
                ba_change = dx_in(10:12);
                ba_norm = norm(ba_change);
                if ba_norm > 0.5  % 加速度バイアス [m/s^2]
                    dx_out(10:12) = ba_change * (0.5 / ba_norm);
                end
                
                bg_change = dx_in(13:15);
                bg_norm = norm(bg_change);
                if bg_norm > 0.1  % ジャイロバイアス [rad/s]
                    dx_out(13:15) = bg_change * (0.1 / bg_norm);
                end
            end
        end
        
        function [vel_out, P_out, was_clipped] = check_and_clip_velocity(obj, vel_in, P_in, vel_indices)
            %check_and_clip_velocity 速度のチェックとクリッピング
            %   入力:
            %     vel_in: 速度ベクトル [3x1]
            %     P_in: 共分散行列
            %     vel_indices: Pにおける速度のインデックス (デフォルト: 4:6)
            %   出力:
            %     vel_out: クリッピング後の速度
            %     P_out: 更新された共分散行列
            %     was_clipped: クリッピングが行われたか
            
            if nargin < 4
                vel_indices = 4:6;
            end
            
            vel_out = vel_in;
            P_out = P_in;
            was_clipped = false;
            
            vel_norm = norm(vel_in);
            
            if vel_norm > obj.max_velocity
                % 速度をクリッピング
                vel_out = vel_in * (obj.max_velocity / vel_norm);
                was_clipped = true;
                
                % 速度分散をリセット (インフレートではなく)
                vel_var_reset = 0.01;  % [m^2/s^2]
                P_out(vel_indices, vel_indices) = eye(3) * vel_var_reset;
            end
        end
        
        function [accel_out, was_clipped] = clip_acceleration(obj, accel_in, dt)
            %clip_acceleration 加速度のクリッピング
            %   入力:
            %     accel_in: 加速度ベクトル [3x1]
            %     dt: タイムステップ
            %   出力:
            %     accel_out: クリッピング後の加速度
            %     was_clipped: クリッピングが行われたか
            
            accel_out = accel_in;
            was_clipped = false;
            
            accel_norm = norm(accel_in);
            max_accel_in_dt = obj.max_acceleration * dt;
            
            if accel_norm > max_accel_in_dt
                accel_out = accel_in * (max_accel_in_dt / accel_norm);
                was_clipped = true;
            end
        end
        
        function reset_sensor_history(obj, sensor_name)
            %reset_sensor_history 特定センサーの履歴をリセット
            if isfield(obj.prev_innovations, sensor_name)
                obj.prev_innovations = rmfield(obj.prev_innovations, sensor_name);
            end
            if isfield(obj.update_counts, sensor_name)
                obj.update_counts = rmfield(obj.update_counts, sensor_name);
            end
        end
        
        function reset_all_history(obj)
            %reset_all_history 全センサーの履歴をリセット
            obj.prev_innovations = struct();
            obj.update_counts = struct();
        end
        
        function set_thresholds(obj, param_name, value)
            %set_thresholds 閾値の動的変更
            %   例: guard.set_thresholds('max_velocity', 3.0);
            
            if isprop(obj, param_name)
                obj.(param_name) = value;
            else
                warning('DivergenceGuard:InvalidProperty', ...
                    'Property %s does not exist', param_name);
            end
        end
        
        function info = get_status(obj)
            %get_status 現在の状態情報を取得
            info = struct();
            info.sensor_names = fieldnames(obj.update_counts);
            info.update_counts = obj.update_counts;
            info.max_velocity = obj.max_velocity;
            info.max_allowed_innov = obj.max_allowed_innov;
            info.innov_change_ratio_threshold = obj.innov_change_ratio_threshold;
        end
    end
end

function value = get_field(s, field, default)
    %get_field 構造体からフィールドを取得、なければデフォルト値
    if isfield(s, field)
        value = s.(field);
    else
        value = default;
    end
end
