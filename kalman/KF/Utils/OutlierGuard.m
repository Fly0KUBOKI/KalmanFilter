classdef OutlierGuard
    % OutlierGuard  外れ値判定と発散防止のラッパー
    %  このユーティリティは SensorFilter と DivergenceGuard を組み合わせて
    %  各センサー更新での一貫した外れ値判定・減衰・ゲインクランプ・dx クリップを行う

    methods (Static)
        function [should_update, y_used, K_used, dx_used, diagnostics] = checkAndApply(sensor_name, z, h, H, P, R, K_proposed, dx_proposed, divergence_guard, noiseEstimator, ctx)
            % 入力:
            %  sensor_name - 'gps'|'mag'|'baro'|'accel' 等
            %  z,h,H,P,R - イノベーション計算に必要な値（y = z-h）
            %  K_proposed - 事前に計算された Kalman gain（無ければ []）
            %  dx_proposed - 事前に計算された状態変化量（無ければ []）
            %  divergence_guard - DivergenceGuard インスタンス
            %  noiseEstimator - NoiseEstimator インスタンス（SensorFilter と同様に使う）
            %  ctx - 任意の診断コンテキスト
            % 出力:
            %  should_update - 更新を実行すべきか
            %  y_used - フィルタ後のイノベーション
            %  K_used - クランプ後の Kalman gain（計算不可なら []）
            %  dx_used - クリップ後の dx（無ければ dx_proposed を返す）
            %  diagnostics - 構造体（norms, S_rcond, flags 等）

            diagnostics = struct();
            should_update = false;
            y_used = [];
            K_used = [];
            dx_used = dx_proposed;

            % compute innovation y and S if possible
            try
                y = z - h;
                S = H * P * H' + R;
                diagnostics.S_rcond = rcond(S);
            catch
                y = [];
                S = [];
                diagnostics.S_rcond = NaN;
            end

            % 1) SensorFilter による外れ値判定
            try
                if isempty(y)
                    should_update = false;
                    diagnostics.sensor_filter = 'no_y';
                    return;
                end
                [y_filtered, sf_update] = SensorFilter.filterInnovation(y, R);
                diagnostics.sensor_filter = sf_update;
                y_used = y_filtered;
            catch e
                diagnostics.sensor_filter = sprintf('error:%s', e.message);
                should_update = false;
                return;
            end

            if ~sf_update
                should_update = false;
                return;
            end

            % --- Mahalanobis gating (multivariate外れ値判定) ---
            % センサー次元に基づくカイ二乗閾値で判定し、大きければスキップ、やや大きければ縮小適用
            try
                if ~isempty(S)
                    % 数値安定化: まず条件数をチェックして必要ならジッターを追加
                    sensor_dim = length(y_used);
                    diagnostics.S_rcond = NaN;
                    try
                        diagnostics.S_rcond = rcond(S);
                    catch
                        diagnostics.S_rcond = NaN;
                    end

                    S_use = S;
                    % Safety: if measurement noise R (or S) is extremely small compared to P
                    % then inflate S diagonal by a small floor proportional to P's scale.
                    try
                        p_scale = max(1e-8, mean(abs(diag(P))));
                        minR = max(1e-8, 1e-6 * p_scale);
                        % add a small floor to S to avoid near-zero diagonal elements
                        S_use = S_use + eye(size(S_use,1)) * minR;
                        diagnostics.S_minR_added = minR;
                    catch
                        % if anything fails, continue without inflation
                    end
                    % If S appears ill-conditioned, add small jitter proportional to its trace
                    if ~isfinite(diagnostics.S_rcond) || diagnostics.S_rcond < 1e-10
                        nS = size(S,1);
                        base_scale = max(abs(trace(S)) / max(1,nS), 1e-8);
                        jitter = max(1e-8, 1e-6 * base_scale);
                        S_use = S + eye(nS) * jitter;
                        try
                            diagnostics.S_rcond_after = rcond(S_use);
                        catch
                            diagnostics.S_rcond_after = NaN;
                        end
                        diagnostics.S_regularized = true;
                        diagnostics.S_jitter = jitter;
                    else
                        diagnostics.S_regularized = false;
                    end

                    % 安全のため inv は使わず pinv を使って Mahalanobis 距離を計算
                    invS = pinv(S_use);
                    d2 = real(y_used' * (invS * y_used));

                    % カイ二乗閾値（利用できない場合は (3σ)^2 * dim をフォールバック）
                    try
                        chi2_thr = chi2inv(0.975, sensor_dim);
                    catch
                        chi2_thr = (3.0^2) * sensor_dim;
                    end

                    % 閾値設定: ctx による上書きを許可（センサー別調整可）
                    diagnostics.mahal_d2 = d2;
                    diagnostics.mahal_thr = chi2_thr;
                    % defaults
                    default_chi2_skip_mult = 10; % tightened: skip earlier than before
                    % immediate skip multiplier: for sensors that commonly produce large outliers
                    % we may want a smaller multiplier to skip earlier (e.g., mag)
                    if exist('ctx','var') && isstruct(ctx) && isfield(ctx,'outlier_params')
                        op = ctx.outlier_params;
                        if isfield(op,'chi2_skip_mult')
                            chi2_skip_mult = op.chi2_skip_mult;
                        else
                            chi2_skip_mult = default_chi2_skip_mult;
                        end
                        if isfield(op,'immediate_skip_mult')
                            immediate_skip_mult = op.immediate_skip_mult;
                        else
                            immediate_skip_mult = [];
                        end
                    else
                        chi2_skip_mult = default_chi2_skip_mult;
                        immediate_skip_mult = [];
                    end
                    % sensor-specific default immediate multiplier
                    if isempty(immediate_skip_mult)
                        if contains(lower(sensor_name),'mag')
                            immediate_skip_mult = 10; % mag: be stricter
                        else
                            immediate_skip_mult = chi2_skip_mult; % no special immediate skip
                        end
                    end

                    chi2_immediate = chi2_thr * immediate_skip_mult;
                    chi2_skip = max(chi2_thr * chi2_skip_mult, 1e3);

                    % 即時スキップ: moderate-large (but not yet 'strong') な場合に即座に更新をやめる
                    if d2 > chi2_immediate
                        diagnostics.mahal_decision = 'skip_immediate_outlier';
                        diagnostics.mahal_immediate_mult = immediate_skip_mult;
                        should_update = false;
                        return;
                    end

                    % 中程度の外れ値はイノベーション・dx を縮小して適用
                    if d2 > chi2_thr
                        w = sqrt(chi2_thr / max(d2, eps));
                        y_used = y_used * w;
                        if ~isempty(dx_used)
                            dx_used = dx_used * w;
                        end
                        diagnostics.mahal_decision = 'attenuated';
                        diagnostics.mahal_weight = w;
                    else
                        diagnostics.mahal_decision = 'ok';
                    end
                end
            catch e
                % 失敗しても処理を止めない（診断にエラーを残す）
                diagnostics.mahal_error = e.message;
            end

            % 2) DivergenceGuard によるイノベーションキャップ／スキップ／減衰
            try
                % divergence_guard.check_and_attenuate_update は dx を返すため、dx_proposed を更新
                if isempty(dx_proposed)
                    % もし dx が無い（例: KF でまだ計算していない）場合は、K_proposed があれば dx = K*y
                    if ~isempty(K_proposed)
                        dx_fromK = K_proposed * y_used;
                        dx_used = dx_fromK;
                    else
                        dx_used = [];
                    end
                end

                % Attach diagnostics into ctx so any divergence dump saved downstream
                % will include OutlierGuard diagnostics for later inspection.
                try
                    if exist('ctx','var') && isstruct(ctx)
                        ctx.diagnostics = diagnostics;
                    end
                catch
                end

                [dx_after, should_skip, was_attenuated] = divergence_guard.check_and_attenuate_update(sensor_name, y_used, dx_used, ctx);
                diagnostics.divergence = struct('should_skip', should_skip, 'was_attenuated', was_attenuated);
                if should_skip
                    should_update = false;
                    return;
                end
                if ~isempty(dx_after)
                    dx_used = dx_after;
                end
            catch e
                diagnostics.divergence = sprintf('error:%s', e.message);
            end

            % 3) Kalman gain のクランプ（もし K_proposed があれば）
            try
                if ~isempty(K_proposed)
                    K_used = divergence_guard.clamp_gain(K_proposed);
                    diagnostics.K_norm = norm(K_used,'fro');
                else
                    K_used = [];
                end
            catch
                K_used = K_proposed;
                diagnostics.K_norm = NaN;
            end

            % 4) dx のクリッピング
            try
                if ~isempty(dx_used)
                    dx_used = divergence_guard.clip_state_change(dx_used);
                end
            catch
            end

            % 5) 最終決断: 更新実行
            should_update = true;
            diagnostics.y_norm = norm(y_used);
            diagnostics.P_diag = diag(P);
            diagnostics.R_diag = diag(R);
            diagnostics.S_rcond = diagnostics.S_rcond;
        end
    end
end
