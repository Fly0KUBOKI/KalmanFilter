# ESKF段階的MEX移行計画

**作成日**: 2025-12-27  
**目標**: ESKFのMATLAB実装を関数ごとに部分的にMEXに移行し、MATLABと全く同じ実装をMEXに移行する

---

## 📋 現状

### 動作確認済み
- ✅ **c274ff9c85daa163c9309e274e0f7e9804e91d1e**: 正しく推定できている
- ✅ **現在のphase1**: 正しく推定できている

### 問題点
- ⚠️ **phase2のmex実装**: phase1との相違点が多く、原因が特定できない

### 方針
- 既存の計画は全て破棄
- ESKFのmatlab実装を関数ごとに部分的にmexに移行していく
- matlabと全く同じ実装をmexに移行する
- 部分的移行では、移行して、実行、解析
- 問題が無ければ次のphaseへ進む

---

## 🎯 ESKF.mの関数構造

### 主要な関数
1. **`predict(obj, a_meas, w_meas)`** - 予測ステップ
   - 現在: `mex_adaptive_predict`を使用（一部MEX化）
   - 後処理: MATLAB（accel_z_integration, velocity_damping, divergence_guard, P正規化）

2. **`sensor_updates(obj, sensor_type, varargin)`** - センサー更新
   - 前処理: `mex_sensor_preprocessor`（MEX化済み）
   - 更新: `do_cpp_update()` → `mex_meukf_step_v2`（MEX化済み）
   - 後処理: MATLAB（divergence_guard, 状態適用）

3. **`do_cpp_update(obj, sensor_type, meas, sample)`** - C++更新の呼び出し
   - 更新: `mex_meukf_step_v2`（MEX化済み）
   - 後処理: MATLAB（divergence_guard, 状態適用、クォータニオン演算）

4. **`reset(obj, method, varargin)`** - リセット
   - チェック: `mex_filter_management`（一部MEX化）
   - 後処理: MATLAB（P設定、状態リセット）

5. **`zupt(obj, method, varargin)`** - ZUPT
   - 更新: `mex_filter_management`（一部MEX化）
   - チェック: MATLAB

6. **`update_filter(obj, obs, k)`** - メインループ
   - 完全MATLAB実装

---

## 📅 段階的移行計画

### Phase 1: predict()の後処理部分をMEX化 ✅ **完了**

**完了日**: 2025年12月27日

**目標**: `predict()`の後処理部分（accel_z_integration, velocity_damping, divergence_guard, P正規化）をMEX化

**実装内容**:
1. ✅ `mex_eskf_predict_postprocess.cpp`を作成
2. ✅ MATLAB実装をそのままC++に移植（`mexCallMATLAB`を使用してMATLAB関数を直接呼び出し）
3. ✅ `predict()`から呼び出し（環境変数`USE_MEX_PREDICT_POSTPROCESS`で制御）
4. ✅ `run_batch_10sets.m`で環境変数を自動設定

**検証結果**:
- ✅ `run_batch_10sets(true)`で10/10成功を確認
- ✅ Position RMSE: Mean=0.7818m（MATLAB実装と同等）
- ✅ Velocity RMSE: Mean=0.5766 m/s（MATLAB実装と同等）
- ✅ Attitude RMSE: Roll=0.2607°, Pitch=0.2812°, Yaw=0.6052°（MATLAB実装と同等）

**ファイル**:
- `kalman/cpp/MEX/mex_eskf_predict_postprocess.cpp`: MEX実装
- `kalman/ESKF.m` (lines 235-273): MEX/MATLAB切り替えロジック
- `kalman/cpp/markdown/PHASE1_PREDICT_POSTPROCESS_COMPLETE.md`: 完了報告

**重要な学び**:
- MEXファイルの再ビルド時はMATLABを完全に終了する必要がある
- `mexCallMATLAB`を使用することでMATLAB実装との数値精度を保証
- 環境変数の設定が重要（`run_batch_10sets.m`で自動設定）
- phase1（MATLAB実装）と結果を比較

---

### Phase 2: do_cpp_update()の後処理部分をMEX化

**目標**: `do_cpp_update()`の後処理部分（divergence_guard, 状態適用、クォータニオン演算）をMEX化

**対象コード** (ESKF.m: 436-477行):
```matlab
if isstruct(dbg_out) && isfield(dbg_out, 'innov') && isfield(dbg_out, 'H')
    obj.noiseEstimator.estimate(sensor_type, dbg_out.innov, dbg_out.H, obj.P);
end

should_apply_mex_state = true;
if isstruct(dbg_out) && isfield(dbg_out, 'dx')
    dx_in = dbg_out.dx(:);
    ctx = struct('sensor', sensor_type, 'sample', sample);
    [dx_out, should_skip, was_attenuated] = obj.divergence_guard.check_and_attenuate_update(sensor_type, dbg_out.innov, dx_in, ctx);
    if should_skip, return; end

    if was_attenuated && ~isempty(dx_out)
        dx_apply = dx_out(:);
        new_state.p = state.p + dx_apply(1:3);
        new_state.v = state.v + dx_apply(4:6);
        phi = dx_apply(7:9);
        dq = [1; 0.5 * phi(:)];
        q1 = dq; q2 = state.q(:);
        [w1, x1, y1, z1] = deal(q1(1), q1(2), q1(3), q1(4));
        [w2, x2, y2, z2] = deal(q2(1), q2(2), q2(3), q2(4));
        q_new = [w1*w2 - x1*x2 - y1*y2 - z1*z2; ...
                 w1*x2 + x1*w2 + y1*z2 - z1*y2; ...
                 w1*y2 - x1*z2 + y1*w2 + z1*x2; ...
                 w1*z2 + x1*y2 - y1*x2 + z1*w2];
        q_new = q_new / norm(q_new);
        new_state.q = q_new;
        new_state.ba = state.ba + dx_apply(10:12);
        new_state.bg = state.bg + dx_apply(13:15);
        if isfield(new_state, 'P')
            new_state.P = (new_state.P + new_state.P')/2;
        else
            new_state.P = obj.P;
        end
    end
end

if should_apply_mex_state
    [obj.p, obj.v, obj.q, obj.ba, obj.bg] = deal(new_state.p, new_state.v, new_state.q, new_state.ba, new_state.bg);
    if isfield(new_state, 'P')
        obj.P = (new_state.P + new_state.P')/2;
    end
end
```

**実装内容**:
1. `mex_eskf_update_postprocess`を作成
2. MATLAB実装をそのままC++に移植
3. `do_cpp_update()`から呼び出し

**検証**:
- `run_batch_10sets()`で10/10成功を確認
- Phase 1と結果を比較

---

### Phase 3: sensor_updates()の完全MEX化

**目標**: `sensor_updates()`の前処理+更新+後処理を統合してMEX化

**対象コード** (ESKF.m: 284-367行):
- 前処理: `mex_sensor_preprocessor`（既にMEX化済み）
- 更新: `do_cpp_update()`（Phase 2で後処理をMEX化済み）
- 統合: 前処理+更新+後処理を1つのMEX関数に統合

**実装内容**:
1. `mex_eskf_sensor_update`を作成
2. 前処理、更新、後処理を統合
3. `sensor_updates()`から呼び出し

**検証**:
- `run_batch_10sets()`で10/10成功を確認
- Phase 2と結果を比較

---

### Phase 4: reset()の完全MEX化

**目標**: `reset()`のチェック+後処理を統合してMEX化

**対象コード** (ESKF.m: 492-532行):
```matlab
function reset(obj, method, varargin)
    if strcmp(method, 'check')
        % チェックロジック
        reset_needed = mex_filter_management('check_divergence', obj.P);
        if any(isnan([obj.p; obj.v; obj.q])) || any(isinf([obj.p; obj.v])) || ...
            norm(obj.v) > 10.0 || norm(obj.p) > 1000.0
            reset_needed = true;
        end
        if reset_needed
            % リセット処理
        end
    elseif strcmp(method, 'filter')
        % フィルタリセット処理
    end
end
```

**実装内容**:
1. `mex_eskf_reset`を作成
2. MATLAB実装をそのままC++に移植
3. `reset()`から呼び出し

**検証**:
- `run_batch_10sets()`で10/10成功を確認
- Phase 3と結果を比較

---

### Phase 5: zupt()の完全MEX化

**目標**: `zupt()`のチェック+更新を統合してMEX化

**対象コード** (ESKF.m: 534-552行):
```matlab
function varargout = zupt(obj, method, varargin)
    if strcmp(method, 'check')
        a_meas = varargin{1}; w_meas = varargin{2};
        a_norm = norm(a_meas);
        gravity_deviation = abs(a_norm - 9.81);
        w_norm = norm(w_meas);
        if gravity_deviation < obj.zupt_threshold_accel && w_norm < obj.zupt_threshold_gyro
            obj.zupt_counter = obj.zupt_counter + 1;
        else
            obj.zupt_counter = 0;
        end
        obj.is_stationary = (obj.zupt_counter >= obj.zupt_min_duration);
        varargout{1} = obj.is_stationary;
    elseif strcmp(method, 'update')
        if ~obj.is_stationary, return; end
        [v_new, P_new] = mex_filter_management('apply_zupt', obj.v, obj.P);
        [obj.v, obj.P] = deal(v_new, P_new);
    end
end
```

**実装内容**:
1. `mex_eskf_zupt`を作成
2. MATLAB実装をそのままC++に移植
3. `zupt()`から呼び出し

**検証**:
- `run_batch_10sets()`で10/10成功を確認
- Phase 4と結果を比較

---

### Phase 6: update_filter()の完全MEX化（最終目標）

**目標**: `update_filter()`の完全MEX化

**対象コード** (ESKF.m: 271-282行):
```matlab
function update_filter(obj, obs, k)
    a = [obs.accel_x(k); obs.accel_y(k); obs.accel_z(k)];
    w = deg2rad([obs.gyro_x(k); obs.gyro_y(k); obs.gyro_z(k)]);
    obj.predict(a, w);
    obj.sensor_updates('accel', a);
    obj.sensor_updates('mag', [obs.mag_x(k); obs.mag_y(k); obs.mag_z(k)]);
    obj.sensor_updates('baro', obs.baro(k));
    if ~isnan(obs.gps_lat(k)) && ~isnan(obs.gps_lon(k))
        obj.sensor_updates('gps', obs.gps_lat(k), obs.gps_lon(k), obs.gps_alt(k), k);
    end
    obj.reset('check', obs, k);
end
```

**実装内容**:
1. `mex_eskf_update_filter`を作成
2. Phase 1-5でMEX化した関数を統合
3. `update_filter()`から呼び出し

**検証**:
- `run_batch_10sets()`で10/10成功を確認
- Phase 5と結果を比較

---

## 🔧 実装方針

### 各Phaseでの実装手順

1. **MATLAB実装の分析**
   - 対象関数のコードを詳細に確認
   - 依存関係を把握
   - 入力・出力を明確化

2. **MEX関数の作成**
   - MATLAB実装をそのままC++に移植
   - 型変換（double ↔ float）を適切に処理
   - エラーハンドリングを追加

3. **ESKF.mの修正**
   - フラグを追加してMEX/MATLABを切り替え可能にする
   - 例: `obj.options.use_mex_predict_postprocess = true;`

4. **検証**
   - `run_batch_10sets()`で10/10成功を確認
   - 前のPhaseと結果を比較
   - 問題があれば修正して再検証

5. **次のPhaseへ**
   - 問題が無ければ次のPhaseへ進む

### 重要な原則

1. **MATLAB実装をそのまま移植**
   - 数値計算の順序を変えない
   - 条件分岐をそのまま移植
   - 変数名も可能な限り一致させる

2. **段階的な移行**
   - 1つの関数を完全に移行してから次へ
   - 問題が発生したらそのPhaseで詳細に調査

3. **検証の徹底**
   - 各Phaseで必ず`run_batch_10sets()`を実行
   - 前のPhaseと結果を比較
   - 問題があれば修正して再検証

---

## 📊 検証方法

### 各Phaseでの検証

1. **ベースライン確認**
   ```matlab
   % Phase 1（MATLAB実装）で実行
   run_batch_10sets()
   ```

2. **MEX実装の検証**
   ```matlab
   % フラグを有効化して実行
   setenv('USE_MEX_PREDICT_POSTPROCESS', '1');
   run_batch_10sets()
   ```

3. **結果比較**
   ```matlab
   compare_phase_results()
   ```

### 成功基準

- ✅ `run_batch_10sets()`で10/10成功
- ✅ Position RMSE: < 1.0m
- ✅ Velocity RMSE: < 1.0 m/s
- ✅ Attitude RMSE: < 1.0 deg
- ✅ 前のPhaseと結果が一致（許容誤差内）

---

## 📝 実装チェックリスト

### Phase 1: predict()の後処理部分をMEX化
- [ ] `mex_eskf_predict_postprocess.cpp`を作成
- [ ] MATLAB実装をC++に移植
- [ ] `build_mex.m`に追加
- [ ] `ESKF.m`の`predict()`を修正
- [ ] フラグを追加（`USE_MEX_PREDICT_POSTPROCESS`）
- [ ] `run_batch_10sets()`で検証
- [ ] Phase 1（MATLAB実装）と結果を比較

### Phase 2: do_cpp_update()の後処理部分をMEX化
- [ ] `mex_eskf_update_postprocess.cpp`を作成
- [ ] MATLAB実装をC++に移植
- [ ] `build_mex.m`に追加
- [ ] `ESKF.m`の`do_cpp_update()`を修正
- [ ] フラグを追加（`USE_MEX_UPDATE_POSTPROCESS`）
- [ ] `run_batch_10sets()`で検証
- [ ] Phase 1と結果を比較

### Phase 3: sensor_updates()の完全MEX化
- [ ] `mex_eskf_sensor_update.cpp`を作成
- [ ] 前処理+更新+後処理を統合
- [ ] `build_mex.m`に追加
- [ ] `ESKF.m`の`sensor_updates()`を修正
- [ ] フラグを追加（`USE_MEX_SENSOR_UPDATE`）
- [ ] `run_batch_10sets()`で検証
- [ ] Phase 2と結果を比較

### Phase 4: reset()の完全MEX化
- [ ] `mex_eskf_reset.cpp`を作成
- [ ] MATLAB実装をC++に移植
- [ ] `build_mex.m`に追加
- [ ] `ESKF.m`の`reset()`を修正
- [ ] フラグを追加（`USE_MEX_RESET`）
- [ ] `run_batch_10sets()`で検証
- [ ] Phase 3と結果を比較

### Phase 5: zupt()の完全MEX化
- [ ] `mex_eskf_zupt.cpp`を作成
- [ ] MATLAB実装をC++に移植
- [ ] `build_mex.m`に追加
- [ ] `ESKF.m`の`zupt()`を修正
- [ ] フラグを追加（`USE_MEX_ZUPT`）
- [ ] `run_batch_10sets()`で検証
- [ ] Phase 4と結果を比較

### Phase 6: update_filter()の完全MEX化
- [ ] `mex_eskf_update_filter.cpp`を作成
- [ ] Phase 1-5でMEX化した関数を統合
- [ ] `build_mex.m`に追加
- [ ] `ESKF.m`の`update_filter()`を修正
- [ ] フラグを追加（`USE_MEX_UPDATE_FILTER`）
- [ ] `run_batch_10sets()`で検証
- [ ] Phase 5と結果を比較

---

## 🚨 注意事項

1. **型変換**
   - MATLABは`double`、C++内部は`float`
   - MEXラッパーで適切に変換

2. **数値精度**
   - MATLAB実装と全く同じ結果を得ることを目指す
   - 浮動小数点演算の順序を変えない

3. **エラーハンドリング**
   - NaN/Inf検出を追加
   - 適切なエラーメッセージを出力

4. **デバッグ**
   - 各Phaseで詳細なログを出力
   - 問題が発生したらそのPhaseで詳細に調査

---

## 📚 関連ファイル

- `kalman/ESKF.m` - メイン実装
- `kalman/run_simulation.m` - シミュレーション実行
- `kalman/run_batch_10sets.m` - バッチテスト
- `kalman/compare_phase_results.m` - 結果比較
- `kalman/cpp/build/build_mex.m` - MEXビルドスクリプト

---

## 🎯 次のアクション

1. Phase 1の実装を開始
2. `mex_eskf_predict_postprocess.cpp`を作成
3. MATLAB実装をC++に移植
4. 検証を実施

