# MEXフォルダ分離状況の分析

## 分析日時
2026-01-02

## 目的
`kalman/cpp/MEX`フォルダに推定アルゴリズム（カルマンフィルタの実装）が混入していないか、適切に分離できているかを確認する。

## 設計原則（README.mdより）

### 許可されるコード
- ✅ MATLAB配列 ↔ C++型の変換関数
- ✅ `mexFunction`の実装
- ✅ エラーチェックとバリデーション
- ✅ `Inc/`, `Src/`, `Lib/`の関数呼び出し

### 禁止されるコード
- ❌ アルゴリズム実装（カルマンフィルタ計算、行列演算など）
- ❌ ビジネスロジック
- ❌ 状態管理

## ファイル別分析

### 1. `mex_run_eskf.cpp`
**状態**: ✅ 適切に分離されている

- `mexFunction`のみ実装
- すべての処理は`do_init`, `do_step`, `do_get_state`, `do_free`などのヘッダー関数を呼び出す
- アルゴリズム実装なし

### 2. `mex_meukf_step.cpp`
**状態**: ✅ 適切に分離されている

- `mexFunction`のみ実装
- `MEUKFCore::step()`を直接呼び出し（アルゴリズム実装は`Inc/MEUKF/meukf_core.hpp`にあり）
- 型変換と関数呼び出しのみ
- コメントアウトされたコードに行列演算（HPHT計算）があるが、使用されていない

### 3. `mex_eskf_initializer.cpp`
**状態**: ⚠️ 要確認 - 初期化アルゴリズムが含まれている

このファイルには以下の初期化アルゴリズムが含まれています：

1. **統計計算**: `compute_mean`, `compute_std`を使用（`Inc/Common/Math/statistics.hpp`から）
2. **初期姿勢計算**: Roll/Pitch/Yawの計算（`atan2`を使用）
3. **クォータニオン計算**: `Quat::from_euler`を使用
4. **Q行列・P行列の初期化**: 対角要素への値の設定

**判断**:
- これらは「初期化アルゴリズム」であり、推定アルゴリズム（カルマンフィルタの更新ステップ）ではない
- ただし、統計計算や姿勢計算などのビジネスロジックが含まれている
- `compute_mean`, `compute_std`は`Inc/Common/Math/statistics.hpp`の関数を使用しているので、実装は`Inc/`にある
- クォータニオン計算も`Inc/Common/Math/quaternion_lib.hpp`を使用

**推奨事項**:
- このファイルは初期化専用であり、推定アルゴリズム（predict/updateステップ）は含まれていない
- 統計計算や姿勢計算は初期化に必要なビジネスロジックだが、これらは既に`Inc/`の関数を使用している
- より厳密な分離が必要な場合は、初期化アルゴリズムを`Src/ESKF/eskf_initializer.cpp`に移動することを検討

### 4. `Inc/mex_run_eskf_filter_ops.hpp`
**状態**: ✅ 適切に分離されている

- 型変換（double ↔ float）のみ
- `check_state_divergence`, `reset_state_on_divergence`, `ESKFCore::update_zupt`などを呼び出す
- アルゴリズム実装なし

### 5. `Inc/mex_run_eskf_sensor_updates.hpp`
**状態**: ✅ 適切に分離されている

- `preprocess_accel`, `preprocess_mag`, `preprocess_gps`, `preprocess_baro`を呼び出す（`Inc/Common/Sensor/sensor_preprocessor.hpp`から）
- `do_meukf_step`を呼び出し（これは`MEUKFCore::step()`を呼び出す）
- 型変換のみ
- アルゴリズム実装なし

### 6. `Inc/mex_run_eskf_impl.hpp`
**状態**: ✅ 適切に分離されている

- `ESKFRunner::predict`, `MEUKFCore::step`を呼び出す
- 型変換と関数呼び出しのみ
- アルゴリズム実装なし

### 7. `Inc/mex_type_conversion.hpp`
**状態**: ✅ 適切に分離されている

- MATLAB配列とC++配列の型変換のみ
- アルゴリズム実装なし

### 8. `Inc/mex_helpers.hpp`
**状態**: ✅ 適切に分離されている

- MATLAB配列操作のヘルパー関数のみ
- `quat_to_euler`は`Inc/Common/Math/quaternion_lib.hpp`を使用
- アルゴリズム実装なし

## 推定アルゴリズム（カルマンフィルタ）の実装場所

推定アルゴリズムは以下の場所に実装されています（MEXフォルダ外）：

1. **ESKF Predict/Update**: 
   - `Inc/ESKF/eskf_core.hpp` - ESKFCoreクラス
   - `Src/ESKF/eskf_core.cpp` - 実装
   - `Inc/ESKF/eskf_runner.hpp` - ESKFRunnerクラス
   - `Src/ESKF/eskf_runner.cpp` - 実装

2. **MEUKF Step**:
   - `Inc/MEUKF/meukf_core.hpp` - MEUKFCoreクラス
   - `Src/MEUKF/meukf_core.cpp` - 実装

3. **センサー前処理**:
   - `Inc/Common/Sensor/sensor_preprocessor.hpp`
   - `Src/Common/Sensor/sensor_preprocessor.cpp`

4. **フィルター管理**:
   - `Inc/Common/filter_management.hpp`
   - `Src/Common/filter_management.cpp`

## 結論

### 推定アルゴリズムの混入状況
✅ **推定アルゴリズム（カルマンフィルタのpredict/updateステップ）はMEXフォルダに混入していない**

すべての推定アルゴリズムは`Inc/`, `Src/`に適切に配置されており、MEXフォルダからは関数呼び出しのみが行われている。

### 初期化アルゴリズムの状況
⚠️ **`mex_eskf_initializer.cpp`には初期化アルゴリズムが含まれている**

ただし：
- 推定アルゴリズム（predict/update）ではない
- 統計計算は`Inc/Common/Math/statistics.hpp`の関数を使用
- クォータニオン計算は`Inc/Common/Math/quaternion_lib.hpp`を使用
- Q/P行列の初期化は単純な値の設定のみ

### 推奨事項

1. **現状は許容範囲内**: 推定アルゴリズムは適切に分離されており、初期化コードのみがMEXにある
2. **より厳密な分離が必要な場合**: `mex_eskf_initializer.cpp`の初期化ロジックを`Src/ESKF/eskf_initializer.cpp`に移動し、MEXからは呼び出すだけにする

## 確認方法

以下のコマンドで推定アルゴリズムの混入を確認：

```bash
# カルマンフィルタの更新式がMEXにないことを確認
grep -r "K\s*=.*P.*H\|P\s*\+=\|\.transpose()\|\.inverse()" kalman/cpp/MEX

# 実装関数の呼び出しを確認（これらは許可される）
grep -r "ESKFCore::\|MEUKFCore::\|ESKFRunner::" kalman/cpp/MEX
```

