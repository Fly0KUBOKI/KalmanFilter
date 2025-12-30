# MEUKF vs ESKF アルゴリズム比較分析

**作成日**: 2025-12-30  
**目的**: MEUKFとESKFの違いを理解し、統合時の問題を回避する

---

## 概要

現在の実装では、**MEUKF（Multiplicative Extended Unscented Kalman Filter）**と**ESKF（Error State Kalman Filter）**の2つの異なるアルゴリズムが使用されています。

- **MEUKF**: `mex_meukf_step_v2`で使用（センサー更新時に呼び出される）
- **ESKF**: `ESKFCore::update_*`で使用（直接C++実装への置き換え時に使用）

これらは**異なるアルゴリズム**であり、単純に置き換えることはできません。

---

## MEUKF（Multiplicative Extended Unscented Kalman Filter）

### 特徴

1. **Unscented Transformを使用**
   - シグマポイントを生成して非線形関数を通過させる
   - クォータニオンなどの非線形状態を直接扱う

2. **Multiplicative形式**
   - クォータニオンの更新を乗算形式で行う
   - `q_new = q * dq`（dqはエラークォータニオン）

3. **実装場所**
   - `Src/MEUKF/meukf_core.cpp`
   - `MEX/mex_meukf_step.cpp`（MEXラッパー）

### 更新ステップの流れ

```cpp
// MEUKFCore::step() の流れ
1. Prediction Step
   - predict(output.new_state, input.sensor, input.params)

2. Update Step
   - update_accel_meukf()  // Unscented Transformを使用
   - update_mag_meukf()    // Unscented Transformを使用
   - update_gps()          // 線形更新
   - update_baro()         // 線形更新
   - update_zupt()         // 線形更新
```

### バイアス更新の方法

```cpp
// MEUKFでは、dx_fullから直接バイアスを更新
Vector15 dx_full;
// ... K_full計算 ...

// Apply dx_full to p, v, ba, bg
p = p + make_vector3(dx_full(0,0), dx_full(1,0), dx_full(2,0));
v = v + make_vector3(dx_full(3,0), dx_full(4,0), dx_full(5,0));
ba = ba + make_vector3(dx_full(9,0), dx_full(10,0), dx_full(11,0));
bg = bg + make_vector3(dx_full(12,0), dx_full(13,0), dx_full(14,0));
```

### 共分散更新

```cpp
// Joseph formを使用
Matrix15x15 I = Matrix15x15::Identity();
Matrix15x15 I_KH = I - KH_full;
Matrix15x15 P_new = I_KH * P * I_KH.transpose() + K_full * R * K_full.transpose();
```

---

## ESKF（Error State Kalman Filter）

### 特徴

1. **エラー状態を直接推定**
   - 真の状態ではなく、エラー状態（δx）を推定
   - ノミナル状態とエラー状態を分離

2. **線形化アプローチ**
   - エラー状態は小さいと仮定して線形化
   - 拡張カルマンフィルタ（EKF）に近い

3. **実装場所**
   - `Src/ESKF/eskf_core.cpp`
   - `Inc/ESKF/eskf_core.hpp`

### 更新ステップの流れ

```cpp
// ESKFCore::update_*() の流れ
1. 観測モデルの線形化
   - H行列の計算（ヤコビアン）

2. イノベーション計算
   - y = z - h(x)

3. カルマンゲイン計算
   - K = P * H' * (H * P * H' + R)^(-1)

4. エラー状態更新
   - dx = K * y

5. 状態更新
   - update_state_from_dx() でノミナル状態に反映
```

### バイアス更新の方法

```cpp
// ESKFでは、エラー状態dxからバイアスを更新
Vector<15, float> dx;
// ... K計算 ...

// dxから直接バイアスを更新
ba[0] += dx(9, 0);
ba[1] += dx(10, 0);
ba[2] += dx(11, 0);
bg[0] += dx(12, 0);
bg[1] += dx(13, 0);
bg[2] += dx(14, 0);
```

### 共分散更新

```cpp
// Joseph formを使用（MEUKFと同じ）
Matrix<15, 15, float> I = Matrix<15, 15, float>::Identity();
Matrix<15, 15, float> I_KH = I - K * H;
P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
```

---

## 主な違い

### 1. アルゴリズムの根本的な違い

| 項目 | MEUKF | ESKF |
|------|-------|------|
| **アプローチ** | Unscented Transform | 線形化（EKF） |
| **状態表現** | 直接状態（q, p, v, ba, bg） | ノミナル状態 + エラー状態 |
| **非線形性の扱い** | シグマポイントで通過 | ヤコビアンで線形化 |
| **計算コスト** | 高い（シグマポイント生成） | 低い（線形化のみ） |
| **精度** | 非線形性が強い場合に有利 | エラーが小さい場合に有効 |

### 2. バイアス更新の違い

**MEUKF**:
- Unscented Transformを通じて、バイアスを含む全状態を同時に更新
- シグマポイントがバイアスの不確実性を自然に伝播

**ESKF**:
- エラー状態dxから直接バイアスを更新
- 線形化の仮定（エラーが小さい）に依存

### 3. 共分散更新の違い

**MEUKF**:
- シグマポイントを通じて共分散を計算
- 非線形関数の影響をより正確に反映

**ESKF**:
- ヤコビアンを使用して共分散を更新
- 線形化誤差が含まれる可能性

---

## 統合時の問題点

### 問題1: アルゴリズムの不一致

**現象**: ジャイロバイアスが0のまま更新されない

**原因**:
- 元の実装では`mex_meukf_step_v2`（MEUKF）を呼び出していた
- 直接C++実装では`ESKFCore::update_*`（ESKF）を使用
- **MEUKFとESKFでは、バイアス更新の方法が異なる**

**詳細**:
- MEUKFでは、Unscented Transformを通じてバイアスの不確実性が自然に伝播される
- ESKFでは、エラー状態dxから直接バイアスを更新するが、線形化の仮定が満たされていない可能性がある

### 問題2: 前処理・後処理の違い

**MEUKF実装**:
```
mex_sensor_preprocessor → 前処理
mex_meukf_step_v2 → MEUKF更新
mex_eskf_update_postprocess → 後処理
mex_sensor_filter → 発散チェック
```

**ESKF実装（失敗）**:
```
preprocess_* (C++) → 前処理
ESKFCore::update_* → ESKF更新
update_state_from_dx → 状態更新
（発散チェックなし）
```

### 問題3: イノベーション計算の違い

**MEUKF**:
- シグマポイントを通じて観測を予測
- イノベーションが正しく計算される

**ESKF（失敗）**:
- 線形化された観測モデルを使用
- イノベーションが0のまま（Max Innovation: 0.0000）

---

## 推奨される統合アプローチ

### オプション A: MEUKF実装の理解と統合（推奨）

1. **MEUKF実装の詳細分析**
   - `mex_meukf_step_v2`の実装を詳細に分析
   - MEUKFの各ステップを理解

2. **MEUKFを直接C++実装に置き換える**
   - ESKFではなく、MEUKFを使用
   - `MEUKFCore::step()`を直接呼び出す

3. **前処理・後処理の統合**
   - `mex_sensor_preprocessor`をC++実装に置き換え
   - `mex_eskf_update_postprocess`をC++実装に置き換え
   - `mex_sensor_filter`の発散チェックを統合

**メリット**:
- 既存のアルゴリズムを維持
- 精度を維持しながら統合可能

**デメリット**:
- MEUKFの実装が複雑
- 計算コストが高い

### オプション B: 段階的な統合（現実的）

1. **前処理のみ統合**
   - `mex_sensor_preprocessor` → C++実装
   - 更新処理は`mexCallMATLAB`経由のまま

2. **後処理のみ統合**
   - `mex_eskf_update_postprocess` → C++実装
   - 更新処理は`mexCallMATLAB`経由のまま

3. **更新処理は維持**
   - `mex_meukf_step_v2`を`mexCallMATLAB`経由で呼び出す
   - アルゴリズムの変更を避ける

**メリット**:
- リスクが低い
- 段階的に統合可能
- 精度を維持

**デメリット**:
- `mexCallMATLAB`のオーバーヘッドが残る
- 完全な統合にはならない

### オプション C: 完全なESKF移行（長期）

1. **MEUKFからESKFへの完全移行を計画**
2. **ESKF実装を修正**
   - バイアス更新の方法を改善
   - 非線形性の扱いを改善
3. **十分なテストと検証**
   - 精度テストを実施
   - 既存実装との比較

**メリット**:
- 計算コストが低い
- 完全なC++実装

**デメリット**:
- 大規模な変更が必要
   - 精度の低下のリスク
   - 十分なテストが必要

---

## 結論

1. **MEUKFとESKFは異なるアルゴリズム**
   - 単純に置き換えることはできない
   - アルゴリズムの違いを理解することが重要

2. **現在の実装はMEUKFを使用**
   - `mex_meukf_step_v2`が実際の更新処理を担当
   - 精度を維持するため、MEUKFを維持する必要がある

3. **推奨アプローチ**
   - **オプション B（段階的な統合）**が最も現実的
   - 前処理・後処理を統合し、更新処理はMEUKFを維持
   - 将来的に、MEUKFを直接C++実装に置き換えることを検討

---

## 参考資料

- `Src/MEUKF/meukf_core.cpp`: MEUKFの実装
- `MEX/mex_meukf_step.cpp`: MEUKFのMEXラッパー
- `Src/ESKF/eskf_core.cpp`: ESKFの実装
- `05_CURRENT_STATUS_AND_FAILURE_ANALYSIS.md`: 失敗原因の詳細分析

---

**最終更新**: 2025-12-30


