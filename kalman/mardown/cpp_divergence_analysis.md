# C++実装発散の原因分析

**日時:** 2025年12月10日  
**状況:** C++実装(mex_meukf_step_v2)を有効化すると推定精度が著しく悪化

## 観測された症状

### C++実装有効時（use_cpp_meukf = true）
```
Position RMSE: 134.4855 m     ← 目標<2.0m に対して67倍悪化
Velocity RMSE: 2.2567 m/s
Roll RMSE:     116.8764 deg   ← 許容範囲を大幅超過
Pitch RMSE:    36.3020 deg
Yaw RMSE:      104.8714 deg

バイアス推定:
Accel bias (final): [-4.39, 1.59, 24.14] m/s^2  ← 異常に大きい
Gyro bias (final):  [0.70, 0.94, -0.55] rad/s   ← 39.8, 54.0, -31.5 deg/s

総合評価: FAIL
```

### MATLAB実装時（use_cpp_meukf = false）- 参考値
```
Position RMSE: 0.96 m         ← 正常
Velocity RMSE: 0.70 m/s
Roll RMSE:     5.0 deg
Pitch RMSE:    2.6 deg
Yaw RMSE:      3.9 deg

総合評価: PASS
```

## 発散原因の推定

### 1. MEX関数の出力引数不一致（修正済み）
**問題:**
```matlab
% call_meukf_update_cpp.m (L63) - 修正前
[new_state, status, debug_info] = mex_meukf_step_v2(state, sensor, params);
```

**原因:**
- `mex_meukf_step_v2`は1つの出力（state構造体）のみを返す
- 3つの出力を期待していたため「出力引数が不足」エラー発生
- エラーキャッチにより状態更新がスキップされ、予測ステップのみが実行

**影響:**
- 全ての観測更新（accel, mag, GPS, baro）が無効化
- フィルタが観測値を全く使わず発散

**修正:**
```matlab
% call_meukf_update_cpp.m (L63) - 修正後
new_state = mex_meukf_step_v2(state, sensor, params);
```

### 2. MEX入力構造体の不整合の可能性

**懸念点:**
```matlab
% call_meukf_update_cpp.m で構築される構造体
sensor_data.accel = sensor.accel(:);
sensor_data.gyro = sensor.gyro(:);
sensor_data.mag = sensor.mag(:);
sensor_data.gps_pos = sensor.gps_pos(:);
sensor_data.alt_baro = sensor.alt_baro;
sensor_data.dt = sensor.dt;
sensor_data.update_accel = strcmp(update_type, 'accel');
sensor_data.update_mag = strcmp(update_type, 'mag');
sensor_data.update_gps = strcmp(update_type, 'gps');
sensor_data.update_baro = strcmp(update_type, 'baro');
```

**C++側で期待される構造体:**
- `mex_meukf_step_v2.cpp`での構造体フィールド抽出が正しく行われているか？
- update_accel等のブール値が`uint8_t`として正しく変換されているか？

**検証方法:**
1. MEX関数内部でフィールド名をログ出力
2. 各フィールドのデータ型を確認
3. update_flagsが正しく伝達されているか確認

### 3. 座標系・符号の不一致

**懸念点:**
- MATLAB実装とC++実装で座標系定義が異なる可能性
- クォータニオンの積の順序（`q_new = q * dq` vs `q_new = dq * q`）
- 回転行列の転置の有無

**検証方法:**
```matlab
% 簡単なテストケース
q_init = [1; 0; 0; 0];  % 単位クォータニオン
a_meas = [0; 0; 9.81];  % 静止時の加速度（重力のみ）

% MATLAB実装とC++実装で同じ入力を与えて出力を比較
```

### 4. 数値精度の問題（float vs double）

**状況:**
- C++コードはfloat→double変換済み（2025-12-09）
- しかし、MEXインターフェースで型変換が正しく行われているか不明

**確認ポイント:**
```cpp
// mex_meukf_step_v2.cpp
// MATLABのdouble配列をC++のdoubleとして正しく読み取っているか？
input.sensor.accel = static_cast<double>(get_field_scalar(...));  // float? double?
```

### 5. 共分散行列の正定値性喪失

**懸念点:**
- C++実装のCholesky分解が失敗している可能性
- 正定値強制(`ensure_positive_definite`)が不十分
- フォールバック処理が適切でない

**検証方法:**
```cpp
// meukf_core.cpp内でログ出力
if (cholesky_failed) {
    fprintf(stderr, "Cholesky failed at step %d, P_min_eig = %f\n", step, min_eigenvalue);
}
```

### 6. ノイズパラメータの不整合

**C++呼び出し側:**
```matlab
% update_accel_meukf_cpp.m
params.noise_accel = R_est_2d(1:3);  % 3要素
```

**C++実装側:**
```cpp
// meukf_core.cpp
// noise_accelを2x2行列として使用？3x3として使用？
```

**影響:**
- ノイズ行列のサイズ不一致でメモリアクセス違反の可能性
- R行列が不正確でフィルタゲインが異常値

## 再現手順

1. `use_cpp_meukf = true`に設定
2. `run_simulation(42, false)`を実行
3. 警告メッセージ観察:
   ```
   warning: C++ MEUKF failed: 1つまたは複数の出力引数が "mex_meukf_step_v2" の呼び出しに割り当てられていません。
   ```
4. `analyze_results`で発散確認

## デバッグ戦略

### Phase 1: 出力引数修正の効果確認（完了）
- [x] `call_meukf_update_cpp.m` L63を修正
- [ ] 修正後の実行結果を確認

### Phase 2: 最小限のテストケース作成
```matlab
% test_cpp_vs_matlab.m
function test_cpp_vs_matlab()
    % 初期状態
    p = [0; 0; 0];
    v = [0; 0; 0];
    q = [1; 0; 0; 0];
    ba = [0; 0; 0];
    bg = [0; 0; 0];
    P = eye(15) * 0.01;
    
    % 加速度観測（静止時）
    a_meas = [0; 0; 9.81];
    
    % MATLAB実装
    eskf_matlab = ESKF();
    eskf_matlab.use_cpp_meukf = false;
    eskf_matlab.p = p; eskf_matlab.v = v; eskf_matlab.q = q;
    eskf_matlab.ba = ba; eskf_matlab.bg = bg; eskf_matlab.P = P;
    eskf_matlab.update_accel_meukf(a_meas);
    
    % C++実装
    eskf_cpp = ESKF();
    eskf_cpp.use_cpp_meukf = true;
    eskf_cpp.p = p; eskf_cpp.v = v; eskf_cpp.q = q;
    eskf_cpp.ba = ba; eskf_cpp.bg = bg; eskf_cpp.P = P;
    eskf_cpp.update_accel_meukf(a_meas);
    
    % 比較
    fprintf('Position diff: %f m\n', norm(eskf_cpp.p - eskf_matlab.p));
    fprintf('Quaternion diff: %f\n', norm(eskf_cpp.q - eskf_matlab.q));
    fprintf('P diff (Frobenius): %f\n', norm(eskf_cpp.P - eskf_matlab.P, 'fro'));
end
```

### Phase 3: MEX関数のログ出力追加
```cpp
// mex_meukf_step_v2.cpp (mexFunction内)
mexPrintf("[DEBUG] Input state.p = [%f, %f, %f]\n", state.p(0), state.p(1), state.p(2));
mexPrintf("[DEBUG] Input sensor.update_accel = %d\n", sensor.update_accel);
mexPrintf("[DEBUG] Input params.noise_accel = [%f, %f, %f]\n", ...);
```

### Phase 4: 段階的検証
1. **Predict のみ**: GPS/Accel/Mag/Baro更新を全て無効化して予測のみ実行
2. **GPS 更新のみ**: 最もシンプルな観測から検証
3. **Accel 更新追加**: 非線形性が低い観測
4. **Mag 更新追加**: 最も複雑な観測

## 暫定対応

### 推奨アクション
1. **MATLAB実装を維持** (`use_cpp_meukf = false`)
2. C++実装は一旦無効化し、詳細デバッグ後に再有効化
3. デバッグ完了までは性能最適化よりも正確性を優先

### 長期的対応
1. MEX関数の単体テストスイート作成
2. C++実装とMATLAB実装の完全一致性の検証
3. CIパイプラインでの自動テスト導入

## 次のステップ

- [x] 出力引数修正後の`run_simulation`実行中
- [ ] 結果確認待ち（処理時間が長いため実行中）
- [ ] 結果がまだ発散する場合、test_cpp_vs_matlab.mで詳細比較
- [ ] MEX関数にデバッグ出力追加してビルド
- [ ] 最小限のテストケースで動作確認
- [ ] 問題箇所を特定して修正

## 実行結果

### MATLAB実装（use_cpp_meukf = false）- 2025-12-10実行
```
=== Estimation Errors (after initialization) ===
Position RMSE: 1.9016 m      ← 正常（目標<2.0m達成）
Velocity RMSE: 0.5064 m/s
Roll RMSE:     0.2455 deg    ← 優秀
Pitch RMSE:    0.3293 deg
Yaw RMSE:      21.9454 deg   ← やや大きいが許容範囲

Position Max Error: 2.9405 m
Velocity Max Error: 1.3031 m/s

=== Bias Estimation ===
Accel bias (final): [-0.19, -0.01, 0.00] m/s^2  ← 正常範囲
Gyro bias (final):  [-0.01, -0.01, -0.00] rad/s  ← 正常範囲

=== Overall Assessment ===
PASS: All checks passed! ✓
```

### C++実装（use_cpp_meukf = true）- 2025-12-10実行完了
```
=== Estimation Errors (after initialization) ===
Position RMSE: 2.6075 m      ← MATLAB 1.90m から悪化（+37%）
Velocity RMSE: 1.3523 m/s    ← MATLAB 0.51m から悪化（+165%）
Roll RMSE:     0.3293 deg    ← MATLAB 0.25deg と同等
Pitch RMSE:    0.3867 deg    ← MATLAB 0.33deg と同等
Yaw RMSE:      101.6594 deg  ← MATLAB 21.9deg から大幅悪化（+363%）

Position Max Error: 5.2102 m  ← MATLAB 2.94m から悪化（+77%）
Velocity Max Error: 3.4162 m/s

=== Bias Estimation ===
Accel bias (final): [-0.06, 0.30, -0.07] m/s^2  ← やや大きいがMATLABと同程度
Gyro bias (final):  [-0.05, -0.02, 0.26] rad/s  ← 15.1 deg/s（MATLAB -0.6 deg/sから悪化）
Max gyro bias:      0.264 rad/s (15.1 deg/s)    ← バイアス推定が不安定

=== Overall Assessment ===
PASS: All checks passed! ✓（ただし性能は悪化）
```

**注記:** C++実装の実行時間が異常に長い（MATLAB実装の2倍以上）。
これは以下の可能性を示唆：
1. MEX呼び出しのオーバーヘッドが大きい
2. C++内部でループや無駄な計算が発生
3. エラーハンドリングで頻繁に警告が発生（処理が遅延）

## 比較分析

### 改善点
- **発散しない**: 以前のPosition RMSE 134m → 2.6m（大幅改善）
- **Roll/Pitchは正常**: 0.3度程度でMATLAB実装と同等
- **NaN/Inf無し**: 数値的には安定

### 残存問題
1. **Yaw推定の大幅悪化** (101.7° vs 21.9°)
   - 磁気計更新が正しく機能していない可能性
   - update_mag_meukf_cppの実装に問題
   
2. **位置推定の悪化** (2.6m vs 1.9m)
   - GPS更新の精度低下
   - update_gps_cppの実装に問題
   
3. **速度推定の悪化** (1.35 m/s vs 0.51 m/s)
   - 予測ステップまたはGPS更新の問題
   
4. **ジャイロバイアス推定の不安定性** (15.1 deg/s vs 0.6 deg/s)
   - バイアス更新ロジックが異なる可能性

## 原因の絞り込み

### 1. 出力引数修正の効果
**結論:** 修正により発散は解消されたが、精度は依然MATLAB実装に劣る

**証拠:**
- 修正前: Position RMSE 134.5m（完全発散）
- 修正後: Position RMSE 2.6m（機能するが不正確）

### 2. Yaw推定悪化の原因候補

**仮説A: 磁気計更新の実装差異**
```matlab
% MATLAB: update_mag_meukf (ESKF.m L874-1018)
% - イノベーション制限: 0.1 rad
% - マハラノビス棄却: 4.0 sigma
% - dtheta制限: 1.0 deg
% - ノイズ係数: R_est * 1.5

% C++: update_mag_meukf_cpp → call_meukf_update_cpp → mex_meukf_step_v2
% - パラメータが正しく渡されているか？
% - 座標系が一致しているか？
```

**検証方法:**
```matlab
% update_mag_meukf内でデバッグ出力追加
fprintf('[MAG] y_innov = [%f, %f, %f], dtheta = [%f, %f, %f]\n', y, dtheta*180/pi);
```

**仮説B: クォータニオン更新の符号問題**
```matlab
% MATLAB: obj.q = QuaternionLib.multiply(obj.q, dq);
% C++: q_new = quaternion_multiply(q, dq);  or q_new = quaternion_multiply(dq, q); ?
```

### 3. GPS更新悪化の原因候補

**仮説: UKF実装の差異**
- MATLAB版はUKFフォールバック付きEKF
- C++版はどちらの実装？
- パラメータ(alpha, beta, kappa)が一致しているか？

### 4. 速度推定悪化の原因候補

**仮説: 予測ステップの実装差異**
- C++のpredict関数が呼ばれていない可能性
- call_meukf_update_cppは更新のみで予測を含まない

**確認ポイント:**
```matlab
% ESKF.m::predict は別途呼ばれているか？
% C++版mex_meukf_step_v2は予測と更新の両方を実行するのか？
```

## 詳細デバッグ計画

### Priority 1: Yaw推定の修正
1. update_mag_meukf と update_mag_meukf_cpp の出力比較
2. 同じ入力で異なる出力が出るステップを特定
3. C++側のクォータニオン更新ロジック検証

### Priority 2: GPS更新の修正
1. update_gps と update_gps_cpp の出力比較
2. 座標変換(緯度経度→XYZ)が一致しているか確認
3. UKF/EKFフォールバック動作の確認

### Priority 3: 予測ステップの確認
1. predictとMEX呼び出しの関係を整理
2. MEXが予測も含むのか、更新のみなのか明確化
3. 必要に応じて predict_cpp を追加

## 段階的テスト実行中

### テスト計画
1. **Test 1: All MATLAB** - ベースライン確認
2. **Test 2: Accel C++** - 加速度更新のみC++
3. **Test 3: Mag C++** - 磁気計更新のみC++
4. **Test 4: GPS C++** - GPS更新のみC++
5. **Test 5: Baro C++** - 気圧計更新のみC++
6. **Test 6: All C++** - 全てC++

### 実行方法
```matlab
% quick_test_cpp.m を実行
cd /cygdrive/c/Users/takut/OneDrive/ドキュメント/MATLAB/KalmanFilter/kalman
matlab -batch "quick_test_cpp"
```

### 期待される結果
- ベースライン（All MATLAB）: Position RMSE ~1.9m, Yaw RMSE ~22deg
- 問題のあるC++実装を特定: RMSEが大幅に悪化する機能を特定
- 原因の絞り込み: どの更新関数がYaw推定を悪化させるか

**実行状況:** quick_test_cpp.m実行中（バックグラウンド）  
**出力ファイル:** /tmp/quick_test_output.txt

## 関連ファイル

- `kalman/ESKF/ESKF.m` (L296: use_cpp_meukf flag)
- `kalman/ESKF/call_meukf_update_cpp.m` (L63: MEX呼び出し)
- `kalman/cpp/MEX/mex_meukf_step_v2.cpp` (MEXインターフェース)
- `kalman/cpp/MEUKF/meukf_core.cpp` (計算コア)
- `kalman/cpp/MEUKF/meukf_core.hpp` (型定義)
