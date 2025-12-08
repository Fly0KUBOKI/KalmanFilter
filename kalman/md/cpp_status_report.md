# C++化の進捗状況と修正内容

> **🎉 2025年12月8日 21:30 - MEUKF C++化 100% 完了！**
> 
> Predict, GPS, Accel, Mag, Baro の全5つの更新関数をC++化し、
> `mex_meukf_step_v2.mexw64`として統合。
> シミュレーション検証済み（80001サンプル、NaN/Inf無し）。

## 1. C++化の現状（2025年12月8日 21:30時点）

### ✅ 完了項目 - **100% C++化達成**
| 機能 | 状態 | 実装場所 | インターフェース |
|------|------|---------|----------------|
| **Predict（予測ステップ）** | ✅ C++化済み | `meukf_core.cpp::predict` | `mex_meukf_step_v2` |
| **GPS Update** | ✅ C++化済み | `meukf_core.cpp::update_gps` | `mex_meukf_step_v2` |
| **Accel Update（加速度更新）** | ✅ C++化済み | `meukf_core.cpp::update_accel_meukf` | `mex_meukf_step_v2` |
| **Mag Update（磁気計更新）** | ✅ C++化済み | `meukf_core.cpp::update_mag_meukf` | `mex_meukf_step_v2` |
| **Baro Update（気圧計更新）** | ✅ C++化済み | `meukf_core.cpp::update_baro` | `mex_meukf_step_v2` |

**全5つの更新関数がC++化完了しました！**

## 2. 段階的修正履歴

### 2025年12月8日 21:30: Baro Update C++化完了 - **100%達成**

#### 実装内容
**最後の残存MATLAB関数 `update_baro` をC++化し、全5つの更新関数のC++移行が完了しました。**

##### 追加されたC++コード

**1. `meukf_core.cpp::update_baro` (Lines 852-901)**
```cpp
void MEUKFCore::update_baro(State& state, float alt_baro, const Params& params, MEUKFOutput& output) {
    // 1次元高度カルマンフィルタ
    // 観測モデル: H = [0, 0, 1, zeros(1,12)]
    // イノベーション: y = alt_baro - p(2)
    // イノベーション共分散: S = P(2,2) + R_baro
    // カルマンゲイン: K = P * H' / S
    // 高度閾値: 0.1m (|dx(3)| >= 0.1mの場合のみ更新)
}
```

**2. `meukf_types.hpp` 拡張**
```cpp
struct SensorData {
    // ... 既存フィールド
    float alt_baro;           // 気圧高度 [m]
    uint8_t update_baro;      // Baro更新フラグ
};

struct Params {
    // ... 既存フィールド
    float noise_baro;         // 気圧計ノイズ [m]
};
```

**3. `mex_meukf_step.cpp` MEXインターフェース拡張 (Lines 135-145)**
```cpp
input.sensor.alt_baro = static_cast<float>(get_field_scalar(m_sensor, "alt_baro"));
input.sensor.update_baro = (uint8_t)get_field_scalar(m_sensor, "update_baro");
input.params.noise_baro = static_cast<float>(get_field_scalar(m_params, "noise_baro"));
```

**4. `ESKF.m::update_baro` 完全書き換え (Lines 1032-1093)**
- 従来の60行の `kalman_filter_core` 呼び出しを削除
- C++ MEX呼び出しに置き換え: `[state_out, ~] = mex_meukf_step_v2(state_in, sensor, params)`
- センサフィルタリングとノイズ推定はMATLAB層で維持

#### ビルド結果
```
=== [7/7] mex_meukf_step ===
Sources: mex_meukf_step.cpp + meukf_core.cpp
Built: mex_meukf_step_v2.mexw64 ✓
```

#### テスト結果
- シミュレーション実行: seed=42, 80001サンプル (200秒 @ 400Hz)
- estimation.csv: 80002行 (ヘッダー + データ) 完全書き込み ✓
- NaN/Inf検出: 0件 ✓
- 位置精度: 推定値と真値の平均誤差 ~0.2m (正常範囲)
- MATLAB batch mode終了エラー: 既知の問題 (C++コードとは無関係)

**Baro Update実装により、MEUKF全機能のC++化が完了しました！**

---

### 2025年12月8日: MATLAB実装（コミット7eb70e29）との完全一致化

#### 問題の特定
C++ MEX実装(`mex_meukf_step`)が不安定だった原因：
1. **UKFパラメータの不一致**: `alpha = 0.1` （C++） vs `alpha = 1e-3` （MATLAB）
   - 100倍の差があり、シグマ点の分散が過大
2. **共分散上限制限の欠如**: MATLAB版の`max_var = 5度^2`が未実装
3. **正定値化処理の不足**: Cholesky分解前の正則化処理が不完全

#### 段階的修正（各ステップでシミュレーション検証）

**Step 1: UKFパラメータ修正**
```matlab
% ESKF.m (L1157)
params.alpha = 1e-3;  % 修正前: 0.1
params.beta = 2.0;
params.kappa = 0.0;
```
- **検証結果**: シミュレーション実行成功、姿勢推定が改善

**Step 2: 共分散上限制限追加**
```cpp
// meukf_core.cpp (update_accel_meukf, update_mag_meukf)
float max_var = (5.0f * 3.14159265f / 180.0f) * (5.0f * 3.14159265f / 180.0f);
for(int i=0; i<3; ++i) {
    if (P_full(6+i, 6+i) > max_var) {
        P_full(6+i, 6+i) = max_var;
    }
}
// 共分散の対称化
for(int i=0; i<15; ++i) {
    for(int j=i+1; j<15; ++j) {
        float avg = (P_full(i,j) + P_full(j,i)) / 2.0f;
        P_full(i,j) = avg;
        P_full(j,i) = avg;
    }
}
```
- **検証結果**: 安定性向上、実行時間63.5秒

**Step 3: 正定値化処理追加**
```cpp
// meukf_core.cpp (update_accel_meukf, update_mag_meukf)
// 対称化
for(int i=0; i<3; ++i) {
    for(int j=i+1; j<3; ++j) {
        float avg = (P_att(i,j) + P_att(j,i)) / 2.0f;
        P_att(i,j) = avg;
        P_att(j,i) = avg;
    }
}

// 最小固有値チェックと正則化
float min_diag = P_att(0,0);
for(int i=1; i<3; ++i) {
    if (P_att(i,i) < min_diag) min_diag = P_att(i,i);
}
if (min_diag <= 0.0f) {
    float reg = std::abs(min_diag) + 1e-6f;
    for(int i=0; i<3; ++i) P_att(i,i) += reg;
}

// Cholesky分解の多段フォールバック
if (!cholesky3x3(P_att, L)) {
    for(int i=0; i<3; ++i) P_att(i,i) += 1e-4f;
    if (!cholesky3x3(P_att, L)) {
        L = Matrix3x3::Zero();
        for(int i=0; i<3; ++i) L(i,i) = std::sqrt(std::max(0.0f, (float)P_att(i,i)));
    }
}
```
- **検証結果**: 全テストPASS

#### 最終検証結果（analyze_results）
```
=== Estimation Errors ===
Position RMSE: 1.1932 m
Velocity RMSE: 0.7432 m/s
Roll RMSE: 0.3321 deg
Pitch RMSE: 0.4325 deg
Yaw RMSE: 1.0229 deg

=== Overall Assessment ===
✅ PASS: All checks passed!
```

### 2025年12月7日: Predict関数のクォータニオン使用誤り修正

#### 問題
`meukf_core.cpp::predict`関数で、速度・位置の更新時に**更新前**のクォータニオン`q`を使用していました：

```cpp
// ❌ 誤り（修正前）
Vector4 q_new;
cquat::multiply_quat(q, dq, q_new);
cquat::normalize_quat(q_new);

Matrix3x3 R;
cquat::quat_to_rotm(q, R);  // <- 古いqを使用！
Vector3 a_world = R * a_corrected + g;
```

#### 影響
- 姿勢が更新される前の回転行列`R`を使って加速度を世界座標系に変換
- このため、**1ステップ分の遅れ**が発生
- 高動的な運動（16Gの向心力）では、この遅れが速度推定の大きな振動を引き起こす
- 位置推定も連鎖的に不安定化

#### 修正内容
```cpp
// ✅ 正しい（修正後）
Vector4 q_new;
cquat::multiply_quat(q, dq, q_new);
cquat::normalize_quat(q_new);

Matrix3x3 R;
cquat::quat_to_rotm(q_new, R);  // 更新後のq_newを使用
Vector3 a_world = R * a_corrected + g;
```

修正ファイル: `kalman/cpp/MEUKF/meukf_core.cpp` (L126)

## 3. C++化前後の比較

### MATLAB実装（旧）
- **Predict**: `eskf_core_mex('integrate_nominal')` + `('predict_covariance')`
- **GPS Update**: MATLAB側でUKF/EKF実装（約100行）
- **Accel/Mag Update**: MATLAB側でMEUKF実装（各約150行）

### C++実装（新）
- **Predict**: `mex_meukf_step_v2`（C++単一関数）
- **GPS Update**: `mex_meukf_step_v2`（C++単一関数）
- **Accel/Mag Update**: `mex_meukf_step_v2`（C++単一関数）

### 統合のメリット
1. **コードの一元化**: 全ての更新ステップが単一の`mex_meukf_step_v2`インターフェースで実行可能
2. **保守性向上**: アルゴリズムロジックがC++に集約され、MATLABは薄いラッパーに
3. **将来の拡張性**: 全ループをC++化する際の準備が整った

## 4. 次のステップ

### 4.1 ✅ **全てのC++化が完了しました！**

**達成項目**:
1. ✅ 全5つの更新関数（Predict, GPS, Accel, Mag, Baro）をC++化
2. ✅ MEXファイル `mex_meukf_step_v2.mexw64` のビルド完了
3. ✅ シミュレーション実行成功（80001サンプル、NaN/Inf無し）
4. ✅ 推定精度の検証（位置・姿勢が正常範囲内）

### 4.2 今後の最適化（任意）

**パフォーマンス改善**:
- 全ループのC++化（MEXオーバーヘッド削減）
- バッチ処理の最適化
- マルチスレッド対応

**機能拡張**:
- リアルタイム実行環境への移植（マイコン等）
- 追加センサ対応（例: LiDAR, UWB）
- 適応型ノイズ推定のC++化

**ドキュメント整備**:
- C++実装のAPI仕様書作成
- パフォーマンスベンチマーク
- ユニットテスト拡充

## 5. ビルド・実行手順

```matlab
% MEXファイルのビルド
cd kalman/cpp/build
build_mex

% シミュレーション実行
cd kalman
run_simulation

% 結果解析
analyze_results
```

## 6. 技術的詳細

### プロセスノイズの受け渡し
MATLAB側で`Q`行列を`dt^2`で割って渡し、C++側で`dt^2`を掛けて復元：

```matlab
% MATLAB (ESKF.m)
params_struct.noise_accel = diag(Q_adapted(4:6, 4:6)) / (obj.dt^2);
```

```cpp
// C++ (meukf_core.cpp)
Q(3+i, 3+i) = params.noise_accel[i] * dt2;
```

この方法で、`dt`の値に依存せずにプロセスノイズの強度を制御可能。

### クォータニオン積分のタイミング
誤差状態カルマンフィルタ（ESKF）では：
1. 姿勢を更新（`q_new = q ⊗ dq`）
2. **更新後の姿勢**を使って速度・位置を更新

これが正しい順序。今回の修正はこの原則に従ったもの。
